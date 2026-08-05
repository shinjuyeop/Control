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
  bool handshake_complete = false;

  for (;;) {
    HilState_t snapshot;
    if (!HilState_GetSnapshot(&snapshot, 5u)) {
      osDelay(HIL_EZ_TASK_PERIOD_MS);
      continue;
    }

    HilState_t *shared = HilState_Lock(2u);
    if (shared != NULL) {
      shared->ez_can_healthy = s_initialized && !s_rx_overflow;
      HilState_Unlock();
    }
    s_rx_overflow = false;

    HilEZFrame_t frame;
    while (pop_frame(&frame)) {
      if (!snapshot.ez_can_enabled) continue;
      shared = HilState_Lock(2u);
      if (shared != NULL) {
        shared->ez_rx_count++;
        if (!HilLogic_IsFilled(frame.data, 0xAAu)) {
          shared->ez_running = (frame.data[4] & 0x01u) != 0u;
          shared->ez_speed_control = (frame.data[4] & 0x02u) != 0u;
        }
        HilState_Unlock();
      }
      if (HilLogic_IsFilled(frame.data, 0xAAu)) handshake_complete = true;
      /* Normal command fields are intentionally accepted but need no plant input
       * in this first emulator; status remains controlled by HilState. */
    }

    if (!snapshot.ez_can_enabled) {
      handshake_complete = false;
      s_state = EZ_STATE_WAIT_HANDSHAKE;
      last_tx = osKernelGetTickCount();
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
