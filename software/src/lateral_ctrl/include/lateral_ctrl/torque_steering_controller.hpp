#pragma once

#include <cstdint>
#include <vector>

namespace lateral_ctrl
{
// All angles/rates use the canonical LEFT-positive wheel axis, in rad and rad/s.
// Raw KEYA direction conversion belongs to the actuator boundary, not this class.
struct TorqueSteeringConfig
{
  double full_scale_error_rad{0.5 * 3.14159265358979323846 / 180.0};
  double kd{1500.0};  // raw / (rad/s)
  int32_t peak_limit_raw{2000};
  int32_t sustained_limit_raw{1000};
  double peak_duration_sec{0.2};
  double peak_recovery_sec{2.0};
  double settle_angle_rad{0.05 * 3.14159265358979323846 / 180.0};
  double settle_rate_rad_s{0.5 * 3.14159265358979323846 / 180.0};
  double rate_filter_hz{10.0};
  double soft_limit_margin_rad{1.0 * 3.14159265358979323846 / 180.0};
  double hard_rate_limit_rad_s{120.0 * 3.14159265358979323846 / 180.0};
  double feedback_timeout_sec{0.1};
  double control_period_sec{0.02};
  // Derived from the measured calibration range, never extrapolated mechanical stops.
  double min_angle_rad{-0.4};
  double max_angle_rad{0.4};
  // Method A: u = J_ff*alpha_ff + P + D + F_c*sign(e).
  // Zero gains keep the existing PD response unchanged.
  double inertia_ff{0.0};  // raw / (rad/s^2)
  double feedforward_limit_raw{1000.0};
  double feedforward_filter_hz{5.0};
  double friction_raw{0.0};
  double friction_deadband_rad{0.1 * 3.14159265358979323846 / 180.0};
  std::vector<double> friction_schedule_speed_mps{};
  std::vector<double> friction_schedule_raw{};
  double frictionRawAt(double vehicle_speed_mps) const;
  void validate() const;
};

enum class TorqueSafetyFault { NONE, INVALID_INPUT, FEEDBACK_TIMEOUT, ANGLE_LIMIT, RATE_LIMIT };
const char * torqueSafetyFaultName(TorqueSafetyFault fault);

struct TorqueSteeringOutput
{
  double target_rad{0.0}, error_rad{0.0};
  double rate_rad_s{0.0}, unfiltered_rate_rad_s{0.0};
  double target_accel_rad_s2{0.0};
  double p_raw{0.0}, d_raw{0.0}, ff_raw{0.0}, friction_raw{0.0}, unsaturated_raw{0.0};
  int32_t command_raw{0};  // canonical axis; not the signed wire command
  int32_t limit_raw{0};
  bool saturated{false}, settled{false}, direction_blocked{false};
  bool rate_ready{false}, peak_limited{false};
  TorqueSafetyFault fault{TorqueSafetyFault::NONE};
};

class TorqueSteeringController
{
public:
  explicit TorqueSteeringController(const TorqueSteeringConfig & config = {});
  // Reconfiguration preserves the peak budget; callers stop output before committing it.
  void configure(const TorqueSteeringConfig & config);
  TorqueSafetyFault observeFeedback(double angle_rad, double sample_time_sec);
  TorqueSteeringOutput update(double target_rad, double target_rate_rad_s,
    double target_accel_rad_s2, double vehicle_speed_mps, double now_sec);
  TorqueSteeringOutput update(double target_rad, double target_rate_rad_s, double now_sec)
  {
    return update(target_rad, target_rate_rad_s, 0.0, 0.0, now_sec);
  }
  // Clear motion history, not peak exposure. Re-arming must not buy a new peak window.
  void reset(double now_sec);
  void idle(double now_sec);
  bool rateReady() const { return rate_ready_; }
  const TorqueSteeringConfig & config() const { return config_; }

private:
  void accountOutput(double now_sec);
  TorqueSteeringConfig config_;
  bool have_sample_{false}, rate_ready_{false}, have_output_time_{false};
  double angle_rad_{0.0}, sample_time_sec_{0.0};
  double rate_rad_s_{0.0}, unfiltered_rate_rad_s_{0.0};
  double output_time_sec_{0.0}, peak_used_sec_{0.0}, recovery_sec_{0.0};
  int32_t last_command_raw_{0};
  TorqueSafetyFault feedback_fault_{TorqueSafetyFault::NONE};
};

// Commands arrive asynchronously to the control tick. Differentiate at their
// timestamps so a held target does not create repeated acceleration impulses.
class TargetAccelerationEstimator
{
public:
  double observe(double target_rad, double stamp_sec, double filter_hz, double max_gap_sec);
  void reset();
  double acceleration() const { return accel_; }

private:
  int samples_{0};
  double target_{0.0}, stamp_{0.0}, dt_{0.0}, rate_{0.0}, accel_{0.0};
};

// The only canonical-to-motor torque sign transform. Reject invalid signs/ranges.
int32_t keyaTorqueCommand(int32_t canonical_command, int direction_sign);
}  // namespace lateral_ctrl
