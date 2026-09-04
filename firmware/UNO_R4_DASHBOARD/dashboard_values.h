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
};

#endif  // DASHBOARD_VALUES_H
