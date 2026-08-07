#include "hil_keya.h"

#include "hil_logic.h"
#include "hil_state.h"
#include "cmsis_os2.h"
#include <math.h>
#include <stdbool.h>
#include <string.h>

#define HIL_KEYA_RX_QUEUE_DEPTH 8u
#define HIL_KEYA_TASK_PERIOD_MS 10u
#define HIL_KEYA_FAULT_CAN_DISCONNECTED (1u << 14)
#define HIL_KEYA_STATUS_DISABLED        (1u << 0)

extern FDCAN_HandleTypeDef hfdcan2;

typedef struct {
  uint8_t data[8];
} HilKEYAFrame_t;

static HilKEYAFrame_t s_rx_queue[HIL_KEYA_RX_QUEUE_DEPTH];
static volatile uint8_t s_rx_head;
static volatile uint8_t s_rx_tail;
static volatile bool s_rx_overflow;
static bool s_initialized;
static bool s_motor_enabled;
static int16_t s_motor_speed_rpm;

static void put_be16(uint8_t *data, int16_t value)
{
  const uint16_t raw = (uint16_t)value;
  data[0] = (uint8_t)(raw >> 8);
  data[1] = (uint8_t)raw;
}

static void put_be32(uint8_t *data, int32_t value)
{
  const uint32_t raw = (uint32_t)value;
  data[0] = (uint8_t)(raw >> 24);
  data[1] = (uint8_t)(raw >> 16);
  data[2] = (uint8_t)(raw >> 8);
  data[3] = (uint8_t)raw;
}

static uint16_t fault_bits(const HilState_t *state)
{
  uint16_t bits = s_motor_enabled ? 0u : HIL_KEYA_STATUS_DISABLED;
  /* Generic KEYA fault injection maps to the manual's CAN-disconnected bit. */
  if (state->keya_fault) bits |= HIL_KEYA_FAULT_CAN_DISCONNECTED;
  return bits;
}

static bool send_frame(uint32_t identifier, const uint8_t data[8])
{
  FDCAN_TxHeaderTypeDef header = {0};
  header.Identifier = identifier;
  header.IdType = FDCAN_EXTENDED_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = FDCAN_DLC_BYTES_8;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_OFF;
  header.FDFormat = FDCAN_CLASSIC_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &header,
                                      (uint8_t *)data) == HAL_OK;
}

static void send_counted(uint32_t identifier, const uint8_t data[8],
                         const HilState_t *state)
{
  if (state->keya_drop_tx || !s_initialized) return;
  if (send_frame(identifier, data)) {
    HilState_t *shared = HilState_Lock(2u);
    if (shared != NULL) {
      shared->keya_tx_count++;
      HilState_Unlock();
    }
  }
}

static bool pop_frame(HilKEYAFrame_t *frame)
{
  const uint8_t tail = s_rx_tail;
  if ((frame == NULL) || (tail == s_rx_head)) return false;
  *frame = s_rx_queue[tail];
  s_rx_tail = (uint8_t)((tail + 1u) % HIL_KEYA_RX_QUEUE_DEPTH);
  return true;
}

static void make_response(const HilKeyaCommand_t *command,
                          const HilState_t *state, uint8_t data[8])
{
  memset(data, 0, 8u);
  data[0] = 0x60u;
  switch (command->type) {
    case HIL_KEYA_COMMAND_ENABLE: data[1] = 0x0Du; data[2] = 0x20u; break;
    case HIL_KEYA_COMMAND_DISABLE: data[1] = 0x0Cu; data[2] = 0x20u; break;
    case HIL_KEYA_COMMAND_POSITION: data[1] = 0x02u; data[2] = 0x20u; break;
    case HIL_KEYA_QUERY_POSITION:
      data[1] = 0x04u; data[2] = 0x21u; data[3] = 0x02u;
      put_be32(&data[4], HilLogic_KeyaDegreesToCount(state->steering_position));
      break;
    case HIL_KEYA_QUERY_SPEED:
      data[1] = 0x03u; data[2] = 0x21u; data[3] = 0x01u;
      put_be32(&data[4], s_motor_speed_rpm);
      break;
    case HIL_KEYA_QUERY_CURRENT:
      data[1] = 0x00u; data[2] = 0x21u; data[3] = 0x01u;
      data[7] = 0u;
      break;
    case HIL_KEYA_QUERY_FAULT: {
      const uint16_t bits = fault_bits(state);
      data[1] = 0x12u; data[2] = 0x21u; data[3] = 0x01u;
      data[4] = (uint8_t)(bits >> 8); data[5] = (uint8_t)bits;
      break;
    }
    case HIL_KEYA_QUERY_VOLTAGE:
      data[1] = 0x0Du; data[2] = 0x21u; data[3] = 0x02u;
      data[7] = 0u; /* No separate KEYA supply state exists in phase 1. */
      break;
    case HIL_KEYA_QUERY_TEMPERATURE:
      data[1] = 0x0Fu; data[2] = 0x21u; data[3] = 0x01u;
      data[6] = 0u; /* No separate KEYA driver temperature state exists. */
      break;
    default: break;
  }
}

static void send_heartbeat(const HilState_t *state)
{
  uint8_t data[8] = {0};
  float heartbeat_position = state->steering_position;
  if (heartbeat_position > 32767.0f) heartbeat_position = 32767.0f;
  if (heartbeat_position < -32768.0f) heartbeat_position = -32768.0f;
  put_be16(&data[0], (int16_t)lroundf(heartbeat_position));
  put_be16(&data[2], s_motor_speed_rpm);
  put_be16(&data[4], 0);
  put_be16(&data[6], (int16_t)fault_bits(state));
  send_counted(HIL_KEYA_HEARTBEAT_ID, data, state);
}

void HilKEYA_Init(void)
{
  FDCAN_FilterTypeDef filter = {0};
  s_rx_head = s_rx_tail = 0u;
  s_rx_overflow = false;
  s_motor_enabled = false;
  s_motor_speed_rpm = 0;
  filter.IdType = FDCAN_EXTENDED_ID;
  filter.FilterIndex = 0u;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = HIL_KEYA_COMMAND_ID;
  filter.FilterID2 = 0x1FFFFFFFUL;
  s_initialized =
      (HAL_FDCAN_ConfigFilter(&hfdcan2, &filter) == HAL_OK) &&
      (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_REJECT_REMOTE,
                                    FDCAN_REJECT_REMOTE) == HAL_OK) &&
      (HAL_FDCAN_Start(&hfdcan2) == HAL_OK) &&
      (HAL_FDCAN_ActivateNotification(&hfdcan2,
          FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0u) == HAL_OK);
}

void HilKEYA_RxCallbackFromISR(FDCAN_HandleTypeDef *hfdcan)
{
  FDCAN_RxHeaderTypeDef header;
  uint8_t data[8];
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK) break;
    if ((header.IdType != FDCAN_EXTENDED_ID) ||
        (header.Identifier != HIL_KEYA_COMMAND_ID) ||
        (header.DataLength != FDCAN_DLC_BYTES_8)) continue;
    const uint8_t next = (uint8_t)((s_rx_head + 1u) % HIL_KEYA_RX_QUEUE_DEPTH);
    if (next == s_rx_tail) {
      s_rx_overflow = true;
      continue;
    }
    memcpy(s_rx_queue[s_rx_head].data, data, sizeof(data));
    s_rx_head = next;
  }
}

void HilKEYA_Task(void)
{
  uint32_t next = osKernelGetTickCount();
  uint32_t last_heartbeat = next - HIL_KEYA_HEARTBEAT_MS;
  uint32_t last_rx = next;
  bool rx_seen = false;

  for (;;) {
    HilState_t snapshot;
    if (!HilState_GetSnapshot(&snapshot, 5u)) {
      osDelay(HIL_KEYA_TASK_PERIOD_MS);
      continue;
    }
    HilState_t *shared = HilState_Lock(2u);
    if (shared != NULL) {
      shared->keya_can_healthy = s_initialized && !s_rx_overflow;
      shared->keya_motor_enabled = s_motor_enabled;
      shared->keya_rx_age_ms = rx_seen ?
          (uint32_t)(osKernelGetTickCount() - last_rx) : UINT32_MAX;
      HilState_Unlock();
    }
    s_rx_overflow = false;

    HilKEYAFrame_t frame;
    while (pop_frame(&frame)) {
      if (!snapshot.keya_can_enabled) continue;
      HilKeyaCommand_t command = HilLogic_KeyaDecode(frame.data);
      if (command.type == HIL_KEYA_COMMAND_INVALID) continue;
      last_rx = osKernelGetTickCount();
      rx_seen = true;
      shared = HilState_Lock(2u);
      if (shared != NULL) {
        shared->keya_rx_count++;
        if (command.type == HIL_KEYA_COMMAND_POSITION) {
          shared->steering_target = HilLogic_KeyaCountToDegrees(command.position_count);
        }
        HilState_Unlock();
      }
      if (command.type == HIL_KEYA_COMMAND_ENABLE) s_motor_enabled = true;
      else if (command.type == HIL_KEYA_COMMAND_DISABLE) s_motor_enabled = false;
      if (HilState_GetSnapshot(&snapshot, 2u)) {
        uint8_t response[8];
        make_response(&command, &snapshot, response);
        send_counted(HIL_KEYA_FEEDBACK_ID, response, &snapshot);
      }
    }

    if (snapshot.keya_can_enabled && s_motor_enabled) {
      const float old_position = snapshot.steering_position;
      const float new_position = HilLogic_Approach(
          old_position, snapshot.steering_target,
          HIL_KEYA_SIM_RATE_DEG_PER_SEC * HIL_KEYA_TASK_PERIOD_MS / 1000.0f);
      const float delta = new_position - old_position;
      s_motor_speed_rpm = (int16_t)lroundf(delta *
          (60000.0f / (360.0f * HIL_KEYA_TASK_PERIOD_MS)));
      shared = HilState_Lock(2u);
      if (shared != NULL) {
        shared->steering_position = new_position;
        HilState_Unlock();
      }
      snapshot.steering_position = new_position;
    } else {
      s_motor_speed_rpm = 0;
    }

    const uint32_t now = osKernelGetTickCount();
    if (!snapshot.keya_can_enabled) {
      s_motor_enabled = false;
      rx_seen = false;
      last_heartbeat = now;
    } else if ((uint32_t)(now - last_heartbeat) >= HIL_KEYA_HEARTBEAT_MS) {
      last_heartbeat += HIL_KEYA_HEARTBEAT_MS;
      send_heartbeat(&snapshot);
    }

    next += HIL_KEYA_TASK_PERIOD_MS;
    (void)osDelayUntil(next);
  }
}
