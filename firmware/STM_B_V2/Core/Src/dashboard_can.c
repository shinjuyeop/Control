#include "dashboard_can.h"

#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

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
volatile uint8_t DASHBOARD_CAN_ALIVE = 0U;

static int16_t speed_to_centi_mps(float speed_mps)
{
  const long scaled = lroundf(speed_mps * 100.0f);
  if (scaled > INT16_MAX) return INT16_MAX;
  if (scaled < INT16_MIN) return INT16_MIN;
  return (int16_t)scaled;
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

  taskENTER_CRITICAL();
  speed_mps = CURRENT_SPEED_FROM_A;
  vehicle_valid = VEHICLE_FRAME_VALID_FROM_A;
  detail = DRIVE_DETAIL_FROM_A;
  detail_valid = DRIVE_DETAIL_FRAME_VALID_FROM_A;
  mode = MODE_FROM_A;
  gear = GEAR_FROM_A;
  estop = ESTOP_STATUS;
  brake = BRAKE_REQUEST;
  taskEXIT_CRITICAL();

  snapshot->alive = alive;
  snapshot->status_flags = 0U;
  snapshot->drive_flags = 0U;
  snapshot->pedal_percent = detail.pedal_percent;
  snapshot->speed_centi_mps = speed_to_centi_mps(speed_mps);
  snapshot->bus_voltage_decivolt = detail.bus_voltage_dv;
  snapshot->bus_current_deciamp = detail.bus_current_da;
  snapshot->phase_current_deciamp = detail.phase_current_da;
  snapshot->controller_temperature_decic =
      detail.controller_temperature_dc;
  snapshot->motor_temperature_decic = detail.motor_temperature_dc;
  snapshot->motor_fault_bits = detail.motor_fault_bits;

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

static uint32_t pack_frame(uint8_t frame_index,
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
  uint8_t slot = 0U;
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
    if (slot == 0U)
    {
      const uint8_t alive = DASHBOARD_CAN_ALIVE++;
      capture_snapshot(&snapshot, alive);
      DASHBOARD_CAN_SNAPSHOT_COUNT++;
    }

    if (slot < DASHBOARD_CAN_FRAME_COUNT)
    {
      uint8_t payload[DASHBOARD_CAN_DLC];
      header.Identifier = pack_frame(slot, &snapshot, payload);

      if (!fdcan_started)
      {
        DASHBOARD_CAN_TX_ERROR_COUNT++;
      }
      else if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan1) == 0U)
      {
        DASHBOARD_CAN_TX_FIFO_FULL_COUNT++;
        DASHBOARD_CAN_TX_ERROR_COUNT++;
      }
      else
      {
        status = HAL_FDCAN_AddMessageToTxFifoQ(
            &hfdcan1, &header, payload);
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
    }

    slot++;
    if (slot >= DASHBOARD_CAN_SLOTS_PER_SNAPSHOT)
    {
      slot = 0U;
    }
    vTaskDelayUntil(
        &last_wake, pdMS_TO_TICKS(DASHBOARD_CAN_SLOT_PERIOD_MS));
  }
}
