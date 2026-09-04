#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "dashboard_can_protocol.h"

int main(void)
{
  DashboardCanSnapshot_t snapshot = {
      .alive = 0xA5U,
      .status_flags = 0xFFU,
      .drive_flags = 0x0FU,
      .pedal_percent = 42U,
      .gnss_state = DASHBOARD_GNSS_RTK_FIXED,
      .speed_centi_mps = -123,
      .bus_voltage_decivolt = 724U,
      .bus_current_deciamp = -183,
      .phase_current_deciamp = 2567,
      .controller_temperature_decic = -55,
      .motor_temperature_decic = 421,
      .motor_fault_bits = 0xAB123456UL};
  uint8_t payload[DASHBOARD_CAN_DLC];

  DashboardCanProtocol_PackStatus(&snapshot, payload);
  assert(memcmp(payload,
                (const uint8_t[]){1U, 0xA5U, 0xFFU, 0x0FU,
                                  42U, DASHBOARD_GNSS_RTK_FIXED,
                                  0x85U, 0xFFU},
                DASHBOARD_CAN_DLC) == 0);
  for (uint8_t state = DASHBOARD_GNSS_UNAVAILABLE;
       state <= DASHBOARD_GNSS_RTK_FIXED;
       ++state)
  {
    snapshot.gnss_state = state;
    DashboardCanProtocol_PackStatus(&snapshot, payload);
    assert(payload[5] == state);
  }

  DashboardCanProtocol_PackPower(&snapshot, payload);
  assert(memcmp(payload,
                (const uint8_t[]){1U, 0xA5U, 0xD4U, 0x02U,
                                  0x49U, 0xFFU, 0x07U, 0x0AU},
                DASHBOARD_CAN_DLC) == 0);

  DashboardCanProtocol_PackThermal(&snapshot, payload);
  assert(memcmp(payload,
                (const uint8_t[]){1U, 0xA5U, 0xC9U, 0xFFU,
                                  0xA5U, 0x01U, 0U, 0U},
                DASHBOARD_CAN_DLC) == 0);

  DashboardCanProtocol_PackFault(&snapshot, payload);
  assert(memcmp(payload,
                (const uint8_t[]){1U, 0xA5U, 0x56U, 0x34U,
                                  0x12U, 0U, 0U, 0U},
                DASHBOARD_CAN_DLC) == 0);

  return 0;
}
