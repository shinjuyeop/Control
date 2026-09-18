#include "dashboard_can_protocol.h"

#include <string.h>

static void put_u16_le(uint8_t destination[2], uint16_t value)
{
  destination[0] = (uint8_t)(value & 0xFFU);
  destination[1] = (uint8_t)(value >> 8);
}

static void begin_payload(const DashboardCanSnapshot_t *snapshot,
                          uint8_t payload[DASHBOARD_CAN_DLC])
{
  memset(payload, 0, DASHBOARD_CAN_DLC);
  payload[0] = DASHBOARD_CAN_PROTOCOL_VERSION;
  payload[1] = snapshot->alive;
}

void DashboardCanProtocol_PackStatus(
    const DashboardCanSnapshot_t *snapshot,
    uint8_t payload[DASHBOARD_CAN_DLC])
{
  begin_payload(snapshot, payload);
  payload[2] = snapshot->status_flags;
  payload[3] = snapshot->drive_flags;
  payload[4] = snapshot->pedal_percent;
  payload[5] = snapshot->gnss_state;
  put_u16_le(&payload[6], (uint16_t)snapshot->speed_centi_mps);
}

void DashboardCanProtocol_PackPower(
    const DashboardCanSnapshot_t *snapshot,
    uint8_t payload[DASHBOARD_CAN_DLC])
{
  begin_payload(snapshot, payload);
  put_u16_le(&payload[2], snapshot->bus_voltage_decivolt);
  put_u16_le(&payload[4], (uint16_t)snapshot->bus_current_deciamp);
  put_u16_le(&payload[6], (uint16_t)snapshot->phase_current_deciamp);
}

void DashboardCanProtocol_PackThermal(
    const DashboardCanSnapshot_t *snapshot,
    uint8_t payload[DASHBOARD_CAN_DLC])
{
  begin_payload(snapshot, payload);
  put_u16_le(&payload[2],
             (uint16_t)snapshot->controller_temperature_decic);
  put_u16_le(&payload[4], (uint16_t)snapshot->motor_temperature_decic);
}

void DashboardCanProtocol_PackFault(
    const DashboardCanSnapshot_t *snapshot,
    uint8_t payload[DASHBOARD_CAN_DLC])
{
  begin_payload(snapshot, payload);
  payload[2] = (uint8_t)(snapshot->motor_fault_bits & 0xFFU);
  payload[3] = (uint8_t)((snapshot->motor_fault_bits >> 8) & 0xFFU);
  payload[4] = (uint8_t)((snapshot->motor_fault_bits >> 16) & 0xFFU);
  payload[5] = snapshot->lv_status_flags;
  payload[6] = snapshot->lv_voltage_decivolt;
  payload[7] = snapshot->lv_soc_percent;
}

void DashboardCanProtocol_PackAuto(
    const DashboardCanSnapshot_t *snapshot,
    uint8_t frame_index,
    uint8_t payload[DASHBOARD_CAN_DLC])
{
  begin_payload(snapshot, payload);
  if (frame_index >= DASHBOARD_CAN_AUTO_FRAME_COUNT)
  {
    return;
  }
  memcpy(&payload[2],
         &snapshot->auto_data[(size_t)frame_index * 6U],
         6U);
  if (frame_index == 0U)
  {
    payload[7] &= (uint8_t)~DASHBOARD_AUTO_HOST_VALID;
    if (snapshot->auto_host_valid != 0U)
    {
      payload[7] |= DASHBOARD_AUTO_HOST_VALID;
    }
  }
}
