#include "hil_ezkontrol.h"

#include "hil_logic.h"
#include "hil_state.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <string.h>

#define HIL_EZ_RX_QUEUE_DEPTH 8u
#define HIL_EZ_TASK_PERIOD_MS 10u

extern FDCAN_HandleTypeDef hfdcan1;

typedef struct {
  uint8_t data[8];
} HilEZFrame_t;

static HilEZFrame_t s_rx_queue[HIL_EZ_RX_QUEUE_DEPTH];
static volatile uint8_t s_rx_head;
static volatile uint8_t s_rx_tail;
static volatile bool s_rx_overflow;
static bool s_initialized;
static HilEZState_t s_state;
static uint8_t s_life;

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
  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &header,
                                      (uint8_t *)data) == HAL_OK;
}

static bool pop_frame(HilEZFrame_t *frame)
{
  const uint8_t tail = s_rx_tail;
  if ((frame == NULL) || (tail == s_rx_head)) return false;
  *frame = s_rx_queue[tail];
  s_rx_tail = (uint8_t)((tail + 1u) % HIL_EZ_RX_QUEUE_DEPTH);
  return true;
}

static void increment_tx_count(void)
{
  HilState_t *state = HilState_Lock(2u);
  if (state != NULL) {
    state->ez_tx_count++;
    HilState_Unlock();
  }
}

void HilEZKontrol_Init(void)
{
  FDCAN_FilterTypeDef filter = {0};
  s_rx_head = s_rx_tail = 0u;
  s_rx_overflow = false;
  s_state = EZ_STATE_WAIT_HANDSHAKE;
  s_life = 0u;

  filter.IdType = FDCAN_EXTENDED_ID;
  filter.FilterIndex = 0u;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = HIL_EZ_COMMAND_ID;
  filter.FilterID2 = 0x1FFFFFFFUL;
  s_initialized =
      (HAL_FDCAN_ConfigFilter(&hfdcan1, &filter) == HAL_OK) &&
      (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT,
                                    FDCAN_REJECT_REMOTE,
                                    FDCAN_REJECT_REMOTE) == HAL_OK) &&
      (HAL_FDCAN_Start(&hfdcan1) == HAL_OK) &&
      (HAL_FDCAN_ActivateNotification(&hfdcan1,
          FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0u) == HAL_OK);
}

void HilEZKontrol_RxCallbackFromISR(FDCAN_HandleTypeDef *hfdcan)
{
  FDCAN_RxHeaderTypeDef header;
  uint8_t data[8];
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u) {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK) break;
    if ((header.IdType != FDCAN_EXTENDED_ID) ||
        (header.Identifier != HIL_EZ_COMMAND_ID) ||
        (header.DataLength != FDCAN_DLC_BYTES_8)) continue;
    const uint8_t next = (uint8_t)((s_rx_head + 1u) % HIL_EZ_RX_QUEUE_DEPTH);
    if (next == s_rx_tail) {
      s_rx_overflow = true;
      continue;
    }
    memcpy(s_rx_queue[s_rx_head].data, data, sizeof(data));
    s_rx_head = next;
  }
}

void HilEZKontrol_Task(void)
{
  uint32_t next = osKernelGetTickCount();
  uint32_t last_tx = next - HIL_EZ_PERIOD_MS;
  uint32_t last_rx = next;
  bool rx_seen = false;
  bool handshake_complete = false;

  for (;;) {
    HilState_t snapshot;
    if (!HilState_GetSnapshot(&snapshot, 5u)) {
      osDelay(HIL_EZ_TASK_PERIOD_MS);
      continue;
    }

    /* Registers only; read outside the mutex to keep the section short. */
    FDCAN_ProtocolStatusTypeDef psr;
    FDCAN_ErrorCountersTypeDef ecr;
    const bool bus_status_ok =
        (HAL_FDCAN_GetProtocolStatus(&hfdcan1, &psr) == HAL_OK) &&
        (HAL_FDCAN_GetErrorCounters(&hfdcan1, &ecr) == HAL_OK);

    HilState_t *shared = HilState_Lock(2u);
    if (shared != NULL) {
      shared->ez_can_healthy = s_initialized && !s_rx_overflow;
      shared->ez_rx_age_ms = rx_seen ?
          (uint32_t)(osKernelGetTickCount() - last_rx) : UINT32_MAX;
      if (bus_status_ok) {
        shared->ez_bus_off = (psr.BusOff != 0u);
        shared->ez_error_passive = (psr.ErrorPassive != 0u);
        shared->ez_warning = (psr.Warning != 0u);
        shared->ez_tec = (uint8_t)ecr.TxErrorCnt;
        shared->ez_rec = (uint8_t)ecr.RxErrorCnt;
      }
      HilState_Unlock();
    }
    s_rx_overflow = false;

    HilEZFrame_t frame;
    while (pop_frame(&frame)) {
      if (!snapshot.ez_can_enabled) continue;
      const HilEZCommand_t command = HilLogic_EZDecodeCommand(frame.data);
      last_rx = osKernelGetTickCount();
      rx_seen = true;
      shared = HilState_Lock(2u);
      if (shared != NULL) {
        shared->ez_rx_count++;
        if (!command.handshake_ack) {
          shared->ez_running = command.running;
          shared->ez_speed_control = command.speed_control;
          shared->ez_command_rpm = command.rpm_field;
          shared->ez_command_current_a = command.phase_current_a;
          shared->ez_target_rpm = HilLogic_EZPlantTargetRpm(&command);
        }
        HilState_Unlock();
      }
      if (command.handshake_ack) handshake_complete = true;
    }

    /* A restarted STM_A waits for a fresh 0x55 request, but the emulator only
     * sends one in WAIT_HANDSHAKE. Without this both sides wait forever. */
    if (handshake_complete &&
        ((uint32_t)(osKernelGetTickCount() - last_rx) >= HIL_EZ_RX_TIMEOUT_MS)) {
      handshake_complete = false;
      shared = HilState_Lock(2u);
      if (shared != NULL) {
        shared->ez_running = false;
        shared->ez_speed_control = false;
        shared->ez_command_rpm = 0.0f;
        shared->ez_command_current_a = 0.0f;
        shared->ez_target_rpm = 0.0f;
        HilState_Unlock();
      }
    }

    if (snapshot.ez_plant_enabled) {
      const float target = handshake_complete ? snapshot.ez_target_rpm : 0.0f;
      const float stepped = HilLogic_PlantStepRpm(
          snapshot.motor_rpm, target,
          (float)HIL_EZ_TASK_PERIOD_MS / 1000.0f, HIL_EZ_PLANT_TAU_S);
      shared = HilState_Lock(2u);
      if (shared != NULL) {
        HilLogic_SetMotionFromRpm(shared, stepped);
        HilState_Unlock();
      }
    }

    if (!snapshot.ez_can_enabled) {
      handshake_complete = false;
      s_state = EZ_STATE_WAIT_HANDSHAKE;
      last_tx = osKernelGetTickCount();
      last_rx = last_tx;
      rx_seen = false;
    } else {
      s_state = !handshake_complete ? EZ_STATE_WAIT_HANDSHAKE :
          (snapshot.ez_fault ? EZ_STATE_FAULT : EZ_STATE_ACTIVE);
      const uint32_t now = osKernelGetTickCount();
      if ((uint32_t)(now - last_tx) >= HIL_EZ_PERIOD_MS) {
        last_tx += HIL_EZ_PERIOD_MS;
        if (snapshot.ez_drop_tx || !s_initialized) {
          /* RX remains enabled while TX is deliberately dropped. */
        } else if (s_state == EZ_STATE_WAIT_HANDSHAKE) {
          uint8_t handshake[8];
          memset(handshake, 0x55, sizeof(handshake));
          if (send_frame(HIL_EZ_STATUS_PART1_ID, handshake)) increment_tx_count();
        } else {
          uint8_t part1[8], part2[8];
          HilLogic_EZEncodePart1(&snapshot, part1);
          HilLogic_EZEncodePart2(&snapshot, s_life++, part2);
          if (send_frame(HIL_EZ_STATUS_PART1_ID, part1)) increment_tx_count();
          if (send_frame(HIL_EZ_STATUS_PART2_ID, part2)) increment_tx_count();
        }
      }
    }

    next += HIL_EZ_TASK_PERIOD_MS;
    (void)osDelayUntil(next);
  }
}
