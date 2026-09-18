#ifndef DASHBOARD_CAN_PROTOCOL_H
#define DASHBOARD_CAN_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

#define DASHBOARD_CAN_PROTOCOL_VERSION 1U
#define DASHBOARD_CAN_DLC 8U

#define DASHBOARD_CAN_STATUS_ID  0x321U
#define DASHBOARD_CAN_POWER_ID   0x322U
#define DASHBOARD_CAN_THERMAL_ID 0x323U
#define DASHBOARD_CAN_FAULT_ID   0x324U
#define DASHBOARD_CAN_AUTO_HEALTH_ID 0x325U
#define DASHBOARD_CAN_AUTO_MOTION_ID 0x326U
#define DASHBOARD_CAN_AUTO_ACTUAL_ID 0x327U
#define DASHBOARD_CAN_AUTO_TARGET_ID 0x328U

#define DASHBOARD_CAN_NATIVE_FRAME_COUNT 4U
#define DASHBOARD_CAN_AUTO_FRAME_COUNT 4U
#define DASHBOARD_CAN_FRAME_COUNT 8U

#define DASHBOARD_AUTO_ROS_VERSION 1U
#define DASHBOARD_AUTO_ROS_PAYLOAD_SIZE 25U
#define DASHBOARD_AUTO_DATA_SIZE 24U
#define DASHBOARD_AUTO_HOST_VALID (1U << 7)

#define DASHBOARD_STATUS_VEHICLE_VALID (1U << 0)
#define DASHBOARD_STATUS_DETAIL_VALID  (1U << 1)
#define DASHBOARD_STATUS_PART1_VALID   (1U << 2)
#define DASHBOARD_STATUS_PART2_VALID   (1U << 3)
#define DASHBOARD_STATUS_MODE_AUTO     (1U << 4)
#define DASHBOARD_STATUS_GEAR_REVERSE  (1U << 5)
#define DASHBOARD_STATUS_ESTOP_ACTIVE  (1U << 6)
#define DASHBOARD_STATUS_BRAKE_ACTIVE  (1U << 7)

#define DASHBOARD_DRIVE_TRACTIVE_FEEDBACK (1U << 0)
#define DASHBOARD_DRIVE_COMMAND_RECEIVED  (1U << 1)
#define DASHBOARD_DRIVE_COMMAND_TIMEOUT   (1U << 2)
#define DASHBOARD_DRIVE_STOP              (1U << 3)

#define DASHBOARD_GNSS_UNAVAILABLE 0U
#define DASHBOARD_GNSS_NO_FIX      1U
#define DASHBOARD_GNSS_FIX         2U
#define DASHBOARD_GNSS_RTK_FIXED   3U

#define DASHBOARD_LV_PRESENT (1U << 0)
#define DASHBOARD_LV_MEASUREMENT_VALID (1U << 1)
#define DASHBOARD_LV_SOC_KNOWN (1U << 2)

typedef struct
{
  uint8_t alive;
  uint8_t status_flags;
  uint8_t drive_flags;
  uint8_t pedal_percent;
  uint8_t gnss_state;
  int16_t speed_centi_mps;
  uint16_t bus_voltage_decivolt;
  int16_t bus_current_deciamp;
  int16_t phase_current_deciamp;
  int16_t controller_temperature_decic;
  int16_t motor_temperature_decic;
  uint32_t motor_fault_bits;
  uint8_t lv_status_flags;
  uint8_t lv_voltage_decivolt;
  uint8_t lv_soc_percent;
  uint8_t auto_data[DASHBOARD_AUTO_DATA_SIZE];
  uint8_t auto_host_valid;
} DashboardCanSnapshot_t;

void DashboardCanProtocol_PackStatus(
    const DashboardCanSnapshot_t *snapshot,
    uint8_t payload[DASHBOARD_CAN_DLC]);

void DashboardCanProtocol_PackPower(
    const DashboardCanSnapshot_t *snapshot,
    uint8_t payload[DASHBOARD_CAN_DLC]);

void DashboardCanProtocol_PackThermal(
    const DashboardCanSnapshot_t *snapshot,
    uint8_t payload[DASHBOARD_CAN_DLC]);

void DashboardCanProtocol_PackFault(
    const DashboardCanSnapshot_t *snapshot,
    uint8_t payload[DASHBOARD_CAN_DLC]);

void DashboardCanProtocol_PackAuto(
    const DashboardCanSnapshot_t *snapshot,
    uint8_t frame_index,
    uint8_t payload[DASHBOARD_CAN_DLC]);

#ifdef __cplusplus
}
#endif

#endif /* DASHBOARD_CAN_PROTOCOL_H */
