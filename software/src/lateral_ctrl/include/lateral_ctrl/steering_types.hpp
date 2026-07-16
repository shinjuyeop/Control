#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>

namespace lateral_ctrl
{
enum class DriveMode : uint8_t { MANUAL = 0, AUTO = 1, UNKNOWN = 255 };
enum class SteeringState : uint8_t {
  INIT, DISABLED, AUTO_READY, ACTIVE, ESTOP, AEB, CAN_LOST, FAULT
};
enum class SteeringDisableReason : uint8_t {
  NONE, MODE_NOT_RECEIVED, MANUAL_MODE, INVALID_MODE, ESTOP, AEB, MOTOR_FAULT,
  CAN_TIMEOUT, FEEDBACK_INVALID
};

struct KeyaMotorState
{
  int32_t motor_position_count{0};
  double motor_speed_rpm{0.0};
  double motor_current_a{0.0};
  double bus_voltage_v{0.0};
  double driver_temperature_c{0.0};
  uint16_t fault_bits{0};
  bool position_valid{false};
  bool speed_valid{false};
  bool current_valid{false};
  bool voltage_valid{false};
  bool temperature_valid{false};
  bool communication_ok{false};
  bool enabled_feedback{false};
  bool enabled_feedback_valid{false};
  std::chrono::steady_clock::time_point last_feedback{};
  std::chrono::steady_clock::time_point last_position_feedback{};
};

struct SafetyInputs
{
  bool real_estop{false};
  bool test_estop{false};
  bool real_aeb{false};
  bool test_aeb{false};
  bool effectiveEstop() const { return real_estop || test_estop; }
  bool effectiveAeb() const { return real_aeb || test_aeb; }
};

inline SteeringDisableReason selectDisableReason(
  bool mode_received, DriveMode mode, const SafetyInputs & safety, bool motor_fault,
  bool can_timeout, bool feedback_valid)
{
  if (safety.effectiveEstop()) return SteeringDisableReason::ESTOP;
  if (can_timeout) return SteeringDisableReason::CAN_TIMEOUT;
  if (motor_fault) return SteeringDisableReason::MOTOR_FAULT;
  if (safety.effectiveAeb()) return SteeringDisableReason::AEB;
  if (!mode_received) return SteeringDisableReason::MODE_NOT_RECEIVED;
  if (mode == DriveMode::MANUAL) return SteeringDisableReason::MANUAL_MODE;
  if (mode != DriveMode::AUTO) return SteeringDisableReason::INVALID_MODE;
  if (!feedback_valid) return SteeringDisableReason::FEEDBACK_INVALID;
  return SteeringDisableReason::NONE;
}

inline SteeringState stateForDisableReason(SteeringDisableReason reason)
{
  if (reason == SteeringDisableReason::NONE) return SteeringState::AUTO_READY;
  if (reason == SteeringDisableReason::ESTOP) return SteeringState::ESTOP;
  if (reason == SteeringDisableReason::AEB) return SteeringState::AEB;
  if (reason == SteeringDisableReason::CAN_TIMEOUT) return SteeringState::CAN_LOST;
  if (reason == SteeringDisableReason::MOTOR_FAULT) return SteeringState::FAULT;
  return SteeringState::DISABLED;
}

inline double rateLimit(
  double previous, double target, double max_rate_deg_s, double dt_sec)
{
  if (!std::isfinite(previous) || !std::isfinite(target) || !std::isfinite(dt_sec) ||
    dt_sec <= 0.0)
  {
    return std::isfinite(previous) ? previous : 0.0;
  }
  const double max_step = std::max(0.0, max_rate_deg_s) * dt_sec;
  return previous + std::clamp(target - previous, -max_step, max_step);
}

inline void resetMotionTargetForRecovery(
  double actual, double & target, double & commanded, bool & target_valid)
{
  target = actual;
  commanded = actual;
  target_valid = false;
}
}  // namespace lateral_ctrl
