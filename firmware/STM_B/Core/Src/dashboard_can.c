#include "runtime_metrics.h"
#include "dashboard_can.h"

#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "board_uart_protocol.h"
#include "dashboard_can_protocol.h"
#include "fdcan.h"
#include "globals.h"
#include "task.h"

volatile uint32_t DASHBOARD_CAN_TX_COUNT = 0U;
volatile uint32_t DASHBOARD_CAN_TX_ERROR_COUNT = 0U;
volatile uint32_t DASHBOARD_CAN_TX_FIFO_FULL_COUNT = 0U;
volatile uint32_t DASHBOARD_CAN_START_ERROR_COUNT = 0U;
volatile uint32_t DASHBOARD_CAN_LAST_HAL_STATUS = HAL_OK;
volatile uint32_t DASHBOARD_CAN_SNAPSHOT_COUNT = 0U;
volatile uint32_t DASHBOARD_CAN_BUS_OFF_COUNT = 0U;
volatile uint32_t DASHBOARD_CAN_RECOVERY_COUNT = 0U;
volatile uint32_t DASHBOARD_CAN_RECOVERY_ERROR_COUNT = 0U;
volatile uint8_t DASHBOARD_CAN_ALIVE = 0U;

static volatile int8_t s_gnss_navsat_status = -1;
static volatile uint32_t s_gnss_last_rx_ms = 0U;
static volatile uint8_t s_gnss_received = 0U;
static uint8_t s_auto_data[DASHBOARD_AUTO_DATA_SIZE];
static volatile uint32_t s_auto_last_rx_ms = 0U;
static volatile uint8_t s_auto_received = 0U;
static uint8_t s_lv_flags = 0U;
static uint8_t s_lv_voltage = 0U;
static uint8_t s_lv_soc = 0xFFU;
static uint32_t s_lv_last_rx_ms = 0U;

void DashboardCan_ObserveLvBattery(const LvBatteryStatus *status)
{
  uint8_t flags = DASHBOARD_LV_PRESENT;
  uint8_t voltage = 0U;
  uint8_t soc = 0xFFU;
  // Conversion into the CAN field's representable 0..25.5 V range.
  if ((status->flags & LV_BATTERY_STATUS_MEASUREMENT_VALID) != 0U &&
      status->bus_voltage_uV >= 0 && status->bus_voltage_uV <= 25500000)
  {
    voltage = (uint8_t)((status->bus_voltage_uV + 50000) / 100000);
    flags |= DASHBOARD_LV_MEASUREMENT_VALID;
  }
  if ((status->flags & LV_BATTERY_STATUS_SOC_KNOWN) != 0U &&
      status->soc_centi_percent >= 0 && status->soc_centi_percent <= 10000)
  {
    soc = (uint8_t)((status->soc_centi_percent + 50) / 100);
    flags |= DASHBOARD_LV_SOC_KNOWN;
  }
  taskENTER_CRITICAL();
  s_lv_flags = flags;
  s_lv_voltage = voltage;
  s_lv_soc = soc;
  s_lv_last_rx_ms = HAL_GetTick();
  taskEXIT_CRITICAL();
}

static int16_t speed_to_centi_mps(float speed_mps)
{
  const long scaled = lroundf(speed_mps * 100.0f);
  if (scaled > INT16_MAX) return INT16_MAX;
  if (scaled < INT16_MIN) return INT16_MIN;
  return (int16_t)scaled;
}

static uint8_t gnss_state_from_status(int8_t navsat_status)
{
  if (navsat_status == 2)
  {
    return DASHBOARD_GNSS_RTK_FIXED;
  }
  if (navsat_status >= 0)
  {
    return DASHBOARD_GNSS_FIX;
  }
  return DASHBOARD_GNSS_NO_FIX;
}

void DashboardCan_ObserveGnssFix(int8_t navsat_status)
{
  const uint32_t now = HAL_GetTick();
  taskENTER_CRITICAL();
  s_gnss_navsat_status = navsat_status;
  s_gnss_last_rx_ms = now;
  s_gnss_received = 1U;
  taskEXIT_CRITICAL();
}

void DashboardCan_ObserveAutoStatus(const uint8_t *payload, size_t size)
{
  if (payload == NULL || size != DASHBOARD_AUTO_ROS_PAYLOAD_SIZE ||
      payload[0] != DASHBOARD_AUTO_ROS_VERSION)
  {
    return;
  }
  taskENTER_CRITICAL();
  memcpy(s_auto_data, &payload[1], DASHBOARD_AUTO_DATA_SIZE);
  s_auto_last_rx_ms = HAL_GetTick();
  s_auto_received = 1U;
  taskEXIT_CRITICAL();
}

static void capture_snapshot(DashboardCanSnapshot_t *snapshot, uint8_t alive)
{
  BoardUartDriveDetail_t detail;
  float speed_mps;
  uint8_t vehicle_valid;
  uint8_t detail_valid;
  uint8_t mode;
  uint8_t gear;
  uint8_t estop;
  uint8_t brake;
  int8_t gnss_navsat_status;
  uint32_t gnss_last_rx_ms;
  uint8_t gnss_received;
  uint32_t auto_last_rx_ms;
  uint8_t auto_received;
  uint32_t lv_last_rx_ms;
  const uint32_t now = HAL_GetTick();

  taskENTER_CRITICAL();
  speed_mps = CURRENT_SPEED_FROM_A;
  vehicle_valid = VEHICLE_FRAME_VALID_FROM_A;
  detail = DRIVE_DETAIL_FROM_A;
  detail_valid = DRIVE_DETAIL_FRAME_VALID_FROM_A;
  mode = MODE_FROM_A;
  gear = GEAR_FROM_A;
  estop = ESTOP_STATUS;
  brake = BRAKE_REQUEST;
  gnss_navsat_status = s_gnss_navsat_status;
  gnss_last_rx_ms = s_gnss_last_rx_ms;
  gnss_received = s_gnss_received;
  memcpy(snapshot->auto_data, s_auto_data, sizeof(snapshot->auto_data));
  auto_last_rx_ms = s_auto_last_rx_ms;
  auto_received = s_auto_received;
  snapshot->lv_status_flags = s_lv_flags;
  snapshot->lv_voltage_decivolt = s_lv_voltage;
  snapshot->lv_soc_percent = s_lv_soc;
  lv_last_rx_ms = s_lv_last_rx_ms;
  taskEXIT_CRITICAL();

  if ((uint32_t)(now - lv_last_rx_ms) >= DASHBOARD_CAN_LV_TIMEOUT_MS)
  {
    snapshot->lv_status_flags = 0U;
    snapshot->lv_voltage_decivolt = 0U;
    snapshot->lv_soc_percent = 0xFFU;
  }

  snapshot->alive = alive;
  snapshot->status_flags = 0U;
  snapshot->drive_flags = 0U;
  snapshot->pedal_percent = detail.pedal_percent;
  snapshot->gnss_state = DASHBOARD_GNSS_UNAVAILABLE;
  snapshot->speed_centi_mps = speed_to_centi_mps(speed_mps);
  snapshot->bus_voltage_decivolt = detail.bus_voltage_dv;
  snapshot->bus_current_deciamp = detail.bus_current_da;
  snapshot->phase_current_deciamp = detail.phase_current_da;
  snapshot->controller_temperature_decic =
      detail.controller_temperature_dc;
  snapshot->motor_temperature_decic = detail.motor_temperature_dc;
  snapshot->motor_fault_bits = detail.motor_fault_bits;
  snapshot->auto_host_valid =
      (auto_received != 0U) &&
      ((uint32_t)(now - auto_last_rx_ms) < DASHBOARD_CAN_AUTO_TIMEOUT_MS);

  if ((gnss_received != 0U) &&
      ((uint32_t)(now - gnss_last_rx_ms) < DASHBOARD_CAN_GNSS_TIMEOUT_MS))
  {
    snapshot->gnss_state = gnss_state_from_status(gnss_navsat_status);
  }

  if (vehicle_valid != 0U)
  {
    snapshot->status_flags |= DASHBOARD_STATUS_VEHICLE_VALID;
  }
  if (detail_valid != 0U)
  {
    snapshot->status_flags |= DASHBOARD_STATUS_DETAIL_VALID;
    if ((detail.flags & BOARD_UART_DETAIL_PART1_VALID) != 0U)
    {
      snapshot->status_flags |= DASHBOARD_STATUS_PART1_VALID;
    }
    if ((detail.flags & BOARD_UART_DETAIL_PART2_VALID) != 0U)
    {
      snapshot->status_flags |= DASHBOARD_STATUS_PART2_VALID;
    }
    if ((detail.flags & BOARD_UART_DETAIL_TRACTIVE_FEEDBACK) != 0U)
    {
      snapshot->drive_flags |= DASHBOARD_DRIVE_TRACTIVE_FEEDBACK;
    }
    if ((detail.flags & BOARD_UART_DETAIL_COMMAND_RECEIVED) != 0U)
    {
      snapshot->drive_flags |= DASHBOARD_DRIVE_COMMAND_RECEIVED;
    }
    if ((detail.flags & BOARD_UART_DETAIL_COMMAND_TIMEOUT) != 0U)
    {
      snapshot->drive_flags |= DASHBOARD_DRIVE_COMMAND_TIMEOUT;
    }
    if ((detail.flags & BOARD_UART_DETAIL_DRIVE_STOP) != 0U)
    {
      snapshot->drive_flags |= DASHBOARD_DRIVE_STOP;
    }
  }
  if (mode == 1U)
  {
    snapshot->status_flags |= DASHBOARD_STATUS_MODE_AUTO;
  }
  if (gear == 1U)
  {
    snapshot->status_flags |= DASHBOARD_STATUS_GEAR_REVERSE;
  }
  if (estop != 0U)
  {
    snapshot->status_flags |= DASHBOARD_STATUS_ESTOP_ACTIVE;
  }
  if (brake != 0U)
  {
    snapshot->status_flags |= DASHBOARD_STATUS_BRAKE_ACTIVE;
  }
}

static uint32_t pack_native_frame(uint8_t frame_index,
                                  const DashboardCanSnapshot_t *snapshot,
                                  uint8_t payload[DASHBOARD_CAN_DLC])
{
  switch (frame_index)
  {
    case 0U:
      DashboardCanProtocol_PackStatus(snapshot, payload);
      return DASHBOARD_CAN_STATUS_ID;
    case 1U:
      DashboardCanProtocol_PackPower(snapshot, payload);
      return DASHBOARD_CAN_POWER_ID;
    case 2U:
      DashboardCanProtocol_PackThermal(snapshot, payload);
      return DASHBOARD_CAN_THERMAL_ID;
    default:
      DashboardCanProtocol_PackFault(snapshot, payload);
      return DASHBOARD_CAN_FAULT_ID;
  }
}

static uint32_t pack_auto_frame(uint8_t frame_index,
                                const DashboardCanSnapshot_t *snapshot,
                                uint8_t payload[DASHBOARD_CAN_DLC])
{
  static const uint32_t identifiers[DASHBOARD_CAN_AUTO_FRAME_COUNT] = {
      DASHBOARD_CAN_AUTO_HEALTH_ID,
      DASHBOARD_CAN_AUTO_MOTION_ID,
      DASHBOARD_CAN_AUTO_ACTUAL_ID,
      DASHBOARD_CAN_AUTO_TARGET_ID};
  DashboardCanProtocol_PackAuto(snapshot, frame_index, payload);
  return identifiers[frame_index];
}

static void enqueue_frame(FDCAN_TxHeaderTypeDef *header,
                          uint32_t identifier,
                          uint8_t payload[DASHBOARD_CAN_DLC],
                          bool fdcan_started)
{
  if (!fdcan_started)
  {
    DASHBOARD_CAN_TX_ERROR_COUNT++;
    return;
  }
  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
  {
    DASHBOARD_CAN_TX_FIFO_FULL_COUNT++;
    DASHBOARD_CAN_TX_ERROR_COUNT++;
    return;
  }
  header->Identifier = identifier;
  const HAL_StatusTypeDef status =
      HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, header, payload);
  DASHBOARD_CAN_LAST_HAL_STATUS = (uint32_t)status;
  if (status == HAL_OK)
  {
    DASHBOARD_CAN_TX_COUNT++;
  }
  else
  {
    DASHBOARD_CAN_TX_ERROR_COUNT++;
  }
}

static bool recover_fdcan(void)
{
  HAL_StatusTypeDef status = HAL_OK;
  if (hfdcan1.State == HAL_FDCAN_STATE_BUSY)
  {
    status = HAL_FDCAN_Stop(&hfdcan1);
  }
  else if (hfdcan1.State != HAL_FDCAN_STATE_READY)
  {
    status = HAL_ERROR;
  }

  if (status == HAL_OK)
  {
    status = HAL_FDCAN_Start(&hfdcan1);
  }
  DASHBOARD_CAN_LAST_HAL_STATUS = (uint32_t)status;
  if (status == HAL_OK)
  {
    DASHBOARD_CAN_RECOVERY_COUNT++;
    return true;
  }
  DASHBOARD_CAN_RECOVERY_ERROR_COUNT++;
  return false;
}

void DashboardCan_Task(void *argument)
{
  FDCAN_TxHeaderTypeDef header = {
      .Identifier = DASHBOARD_CAN_STATUS_ID,
      .IdType = FDCAN_STANDARD_ID,
      .TxFrameType = FDCAN_DATA_FRAME,
      .DataLength = FDCAN_DLC_BYTES_8,
      .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
      .BitRateSwitch = FDCAN_BRS_OFF,
      .FDFormat = FDCAN_CLASSIC_CAN,
      .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
      .MessageMarker = 0U};
  DashboardCanSnapshot_t snapshot = {0};
  TickType_t last_wake = xTaskGetTickCount();
  bool fdcan_started = false;
  bool bus_off_latched = false;
  uint8_t slot = 0U;
  uint32_t last_health_ms = HAL_GetTick();
  uint32_t last_recovery_attempt_ms = HAL_GetTick();
  (void)argument;

  HAL_StatusTypeDef status = HAL_FDCAN_Start(&hfdcan1);
  DASHBOARD_CAN_LAST_HAL_STATUS = (uint32_t)status;
  if (status == HAL_OK)
  {
    fdcan_started = true;
  }
  else
  {
    DASHBOARD_CAN_START_ERROR_COUNT++;
  }

  for (;;)
  {
    RuntimeActivation activation = RuntimeMetrics_Begin(RM_DISPLAY);
    RuntimeMetrics_Release(RM_DISPLAY,
        (int32_t)(activation.tick - last_wake));
    const uint32_t now = HAL_GetTick();
    if (fdcan_started &&
        ((uint32_t)(now - last_health_ms) >=
         DASHBOARD_CAN_HEALTH_PERIOD_MS))
    {
      FDCAN_ProtocolStatusTypeDef protocol_status;
      last_health_ms = now;
      if (HAL_FDCAN_GetProtocolStatus(
              &hfdcan1, &protocol_status) != HAL_OK)
      {
        DASHBOARD_CAN_RECOVERY_ERROR_COUNT++;
      }
      else if (protocol_status.BusOff != 0U)
      {
        if (!bus_off_latched)
        {
          DASHBOARD_CAN_BUS_OFF_COUNT++;
          bus_off_latched = true;
        }
        fdcan_started = false;
        last_recovery_attempt_ms = now;
      }
      else
      {
        bus_off_latched = false;
      }
    }

    if (!fdcan_started &&
        ((uint32_t)(now - last_recovery_attempt_ms) >=
         DASHBOARD_CAN_RECOVERY_BACKOFF_MS))
    {
      last_recovery_attempt_ms = now;
      fdcan_started = recover_fdcan();
      last_health_ms = now;
    }

    if (slot == 0U)
    {
      const uint8_t alive = DASHBOARD_CAN_ALIVE++;
      capture_snapshot(&snapshot, alive);
      DASHBOARD_CAN_SNAPSHOT_COUNT++;
    }

    if (slot < DASHBOARD_CAN_NATIVE_FRAME_COUNT)
    {
      uint8_t native_payload[DASHBOARD_CAN_DLC];
      uint8_t auto_payload[DASHBOARD_CAN_DLC];
      enqueue_frame(&header,
                    pack_native_frame(slot, &snapshot, native_payload),
                    native_payload,
                    fdcan_started);
      enqueue_frame(&header,
                    pack_auto_frame(slot, &snapshot, auto_payload),
                    auto_payload,
                    fdcan_started);
    }

    slot++;
    if (slot >= DASHBOARD_CAN_SLOTS_PER_SNAPSHOT)
    {
      slot = 0U;
    }
    RuntimeMetrics_End(RM_DISPLAY, activation);
    if ((int32_t)(xTaskGetTickCount() - last_wake) >=
        (int32_t)pdMS_TO_TICKS(DASHBOARD_CAN_SLOT_PERIOD_MS))
      RuntimeMetrics_DeadlineMiss(RM_DISPLAY);
    vTaskDelayUntil(
        &last_wake, pdMS_TO_TICKS(DASHBOARD_CAN_SLOT_PERIOD_MS));
  }
}
