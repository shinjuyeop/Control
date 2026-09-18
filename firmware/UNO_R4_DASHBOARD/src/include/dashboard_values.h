#ifndef DASHBOARD_VALUES_H
#define DASHBOARD_VALUES_H

#include <stdint.h>

static constexpr uint8_t STATUS_VEHICLE_VALID = 1U << 0;
static constexpr uint8_t STATUS_DETAIL_VALID = 1U << 1;
static constexpr uint8_t STATUS_PART1_VALID = 1U << 2;
static constexpr uint8_t STATUS_PART2_VALID = 1U << 3;
static constexpr uint8_t STATUS_MODE_AUTO = 1U << 4;
static constexpr uint8_t STATUS_GEAR_REVERSE = 1U << 5;
static constexpr uint8_t STATUS_ESTOP_ACTIVE = 1U << 6;
static constexpr uint8_t STATUS_BRAKE_ACTIVE = 1U << 7;

static constexpr uint8_t DRIVE_TRACTIVE_FEEDBACK = 1U << 0;
static constexpr uint8_t DRIVE_COMMAND_RECEIVED = 1U << 1;
static constexpr uint8_t DRIVE_COMMAND_TIMEOUT = 1U << 2;
static constexpr uint8_t DRIVE_STOP = 1U << 3;

static constexpr uint8_t GNSS_UNAVAILABLE = 0U;
static constexpr uint8_t GNSS_NO_FIX = 1U;
static constexpr uint8_t GNSS_FIX = 2U;
static constexpr uint8_t GNSS_RTK_FIXED = 3U;
static constexpr uint8_t LV_PRESENT = 1U << 0;
static constexpr uint8_t LV_MEASUREMENT_VALID = 1U << 1;
static constexpr uint8_t LV_SOC_KNOWN = 1U << 2;

static constexpr uint8_t AUTO_GPS_WAIT = 0U;
static constexpr uint8_t AUTO_GPS_FIXED = 1U;
static constexpr uint8_t AUTO_GPS_FLOAT = 2U;
static constexpr uint8_t AUTO_GPS_FIX = 3U;
static constexpr uint8_t AUTO_GPS_NO_FIX = 4U;

static constexpr uint8_t AUTO_STREAM_WAIT = 0U;
static constexpr uint8_t AUTO_STREAM_RX = 1U;
static constexpr uint8_t AUTO_STREAM_LOST = 2U;

static constexpr uint8_t AUTO_DRIVE_WAIT = 0U;
static constexpr uint8_t AUTO_DRIVE_RUN = 1U;
static constexpr uint8_t AUTO_DRIVE_HOLD = 2U;
static constexpr uint8_t AUTO_DRIVE_CMD_LOST = 3U;
static constexpr uint8_t AUTO_DRIVE_NO_FEEDBACK = 4U;
static constexpr uint8_t AUTO_DRIVE_RUN_MISMATCH = 5U;
static constexpr uint8_t AUTO_DRIVE_FAULT = 6U;
static constexpr uint8_t AUTO_DRIVE_STATUS_LOST = 7U;
static constexpr uint8_t AUTO_DRIVE_WAIT_CMD = 8U;

static constexpr uint8_t AUTO_STEER_WAIT = 0U;
static constexpr uint8_t AUTO_STEER_ACTIVE = 1U;
static constexpr uint8_t AUTO_STEER_WAIT_CMD = 2U;
static constexpr uint8_t AUTO_STEER_STATUS_LOST = 3U;
static constexpr uint8_t AUTO_STEER_MOTOR_FAULT = 4U;
static constexpr uint8_t AUTO_STEER_CAN_LOST = 5U;
static constexpr uint8_t AUTO_STEER_POS_LOST = 6U;
static constexpr uint8_t AUTO_STEER_CMD_LOST = 7U;
static constexpr uint8_t AUTO_STEER_TRACK_FAULT = 8U;
static constexpr uint8_t AUTO_STEER_TRACK_WARN = 9U;
static constexpr uint8_t AUTO_STEER_BRAKE = 10U;
static constexpr uint8_t AUTO_STEER_ESTOP = 11U;
static constexpr uint8_t AUTO_STEER_DISABLED = 12U;

static constexpr uint8_t AUTO_BRAKE_WAIT = 0U;
static constexpr uint8_t AUTO_BRAKE_OFF = 1U;
static constexpr uint8_t AUTO_BRAKE_ON = 2U;
static constexpr uint8_t AUTO_BRAKE_CMD_LOST = 3U;
static constexpr uint8_t AUTO_BRAKE_STATUS_LOST = 4U;

static constexpr uint8_t AUTO_ACTIVITY_WAIT = 0U;
static constexpr uint8_t AUTO_ACTIVITY_FOLLOW_ROUTE = 1U;
static constexpr uint8_t AUTO_ACTIVITY_REJOIN_ROUTE = 2U;
static constexpr uint8_t AUTO_ACTIVITY_TRAFFIC_CONTROL = 3U;
static constexpr uint8_t AUTO_ACTIVITY_AVOID_OBSTACLE = 4U;
static constexpr uint8_t AUTO_ACTIVITY_INTERSECTION = 5U;
static constexpr uint8_t AUTO_ACTIVITY_FINISH = 6U;
static constexpr uint8_t AUTO_ACTIVITY_EMERGENCY = 7U;
static constexpr uint8_t AUTO_ACTIVITY_ACTIVE = 8U;

static constexpr uint8_t AUTO_OUTPUT_UNKNOWN = 0U;
static constexpr uint8_t AUTO_OUTPUT_OFF = 1U;
static constexpr uint8_t AUTO_OUTPUT_ON = 2U;
static constexpr uint8_t AUTO_FEEDBACK_UNKNOWN = 0U;
static constexpr uint8_t AUTO_FEEDBACK_LOST = 1U;
static constexpr uint8_t AUTO_FEEDBACK_LIVE = 2U;

static constexpr uint16_t AUTO_HEADING_INVALID = 0xFFFFU;
static constexpr int16_t AUTO_I16_INVALID = INT16_MIN;
static constexpr int32_t AUTO_I32_INVALID = INT32_MIN;

struct DashboardValues
{
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
};

struct AutoDashboardValues
{
  uint8_t gps_state;
  uint8_t lidar_state;
  uint8_t camera_left_state;
  uint8_t camera_right_state;
  uint8_t drive_state;
  uint8_t steer_state;
  uint8_t brake_state;
  uint8_t activity;
  uint8_t output_state;
  uint8_t feedback_state;
  uint8_t host_valid;
  uint16_t heading_centideg;
  uint16_t target_heading_centideg;
  int16_t target_speed_centi_kmh;
  int32_t actual_motor_count;
  int32_t target_motor_count;
  int16_t steering_current_deciamp;
};

#endif  // DASHBOARD_VALUES_H
