#include "lateral_ctrl/torque_steering_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace lateral_ctrl
{
void TorqueSteeringConfig::validate() const
{
  for (double value : {full_scale_error_rad, peak_duration_sec, peak_recovery_sec,
      settle_angle_rad, settle_rate_rad_s, rate_filter_hz, soft_limit_margin_rad,
      hard_rate_limit_rad_s, feedback_timeout_sec, control_period_sec})
  {
    if (!std::isfinite(value) || value <= 0.0) {
      throw std::invalid_argument("torque scales, periods and safety limits must be finite and positive");
    }
  }
  if (!std::isfinite(kd) || kd < 0.0 || peak_limit_raw < 1 || peak_limit_raw > 10000 ||
    !std::isfinite(peak_limit_raw / full_scale_error_rad) ||
    sustained_limit_raw < 1 || sustained_limit_raw > peak_limit_raw ||
    !std::isfinite(min_angle_rad) || !std::isfinite(max_angle_rad) ||
    min_angle_rad >= 0.0 || max_angle_rad <= 0.0 ||
    soft_limit_margin_rad >= std::min(-min_angle_rad, max_angle_rad) ||
    settle_angle_rad >= full_scale_error_rad || settle_rate_rad_s >= hard_rate_limit_rad_s ||
    control_period_sec < 0.01 || control_period_sec > 0.05 ||
    feedback_timeout_sec < 2.0 * control_period_sec || feedback_timeout_sec > 0.2 ||
    peak_duration_sec < control_period_sec || peak_duration_sec > 5.0 ||
    peak_recovery_sec < peak_duration_sec || rate_filter_hz > 0.5 / control_period_sec)
  {
    throw std::invalid_argument("inconsistent torque gains, raw limits, timing or calibrated envelope");
  }
  const auto raw_in_range = [](double value) {
      return std::isfinite(value) && value >= 0.0 && value <= 10000.0;
    };
  if (!std::isfinite(inertia_ff) || inertia_ff < 0.0 || !raw_in_range(feedforward_limit_raw) ||
    !std::isfinite(feedforward_filter_hz) || feedforward_filter_hz <= 0.0 ||
    !raw_in_range(friction_raw) ||
    !std::isfinite(friction_deadband_rad) || friction_deadband_rad < 0.0)
  {
    throw std::invalid_argument("feedforward and friction terms must be finite, nonnegative raw values");
  }
  if (friction_schedule_speed_mps.size() != friction_schedule_raw.size()) {
    throw std::invalid_argument("friction schedule speed and raw arrays must have equal length");
  }
  for (std::size_t i = 0; i < friction_schedule_speed_mps.size(); ++i) {
    const double speed = friction_schedule_speed_mps[i];
    if (!std::isfinite(speed) || speed < 0.0 || !raw_in_range(friction_schedule_raw[i]) ||
      (i > 0 && !(speed > friction_schedule_speed_mps[i - 1])))
    {
      throw std::invalid_argument(
        "friction schedule needs strictly increasing nonnegative speeds and raw 0..10000");
    }
  }
}

double TorqueSteeringConfig::frictionRawAt(double vehicle_speed_mps) const
{
  const auto & speeds = friction_schedule_speed_mps;
  if (speeds.empty()) return friction_raw;
  const double speed = std::abs(vehicle_speed_mps);
  if (!(speed > speeds.front())) return friction_schedule_raw.front();
  if (speed >= speeds.back()) return friction_schedule_raw.back();
  const auto upper = static_cast<std::size_t>(
    std::upper_bound(speeds.begin(), speeds.end(), speed) - speeds.begin());
  const double ratio = (speed - speeds[upper - 1]) / (speeds[upper] - speeds[upper - 1]);
  return friction_schedule_raw[upper - 1] +
         ratio * (friction_schedule_raw[upper] - friction_schedule_raw[upper - 1]);
}

const char * torqueSafetyFaultName(TorqueSafetyFault fault)
{
  switch (fault) {
    case TorqueSafetyFault::NONE: return "none";
    case TorqueSafetyFault::INVALID_INPUT: return "invalid_input";
    case TorqueSafetyFault::FEEDBACK_TIMEOUT: return "feedback_timeout";
    case TorqueSafetyFault::ANGLE_LIMIT: return "angle_limit";
    case TorqueSafetyFault::RATE_LIMIT: return "rate_limit";
  }
  return "invalid_input";
}

TorqueSteeringController::TorqueSteeringController(const TorqueSteeringConfig & config)
: config_(config)
{
  config_.validate();
}

void TorqueSteeringController::configure(const TorqueSteeringConfig & config)
{
  config.validate();
  config_ = config;
}

TorqueSafetyFault TorqueSteeringController::observeFeedback(double angle, double sample_time)
{
  if (!std::isfinite(angle) || !std::isfinite(sample_time) ||
    (have_sample_ && (sample_time < sample_time_sec_ ||
    (sample_time == sample_time_sec_ && angle != angle_rad_))))
  {
    return feedback_fault_ = TorqueSafetyFault::INVALID_INPUT;
  }
  if (angle < config_.min_angle_rad || angle > config_.max_angle_rad) {
    return feedback_fault_ = TorqueSafetyFault::ANGLE_LIMIT;
  }
  if (have_sample_ && sample_time == sample_time_sec_) return feedback_fault_;
  feedback_fault_ = TorqueSafetyFault::NONE;
  if (have_sample_) {
    const double dt = sample_time - sample_time_sec_;
    if (dt > config_.feedback_timeout_sec) {
      rate_ready_ = false;
      rate_rad_s_ = unfiltered_rate_rad_s_ = 0.0;
      // A newly arrived sample does not erase an excessive gap between samples.
      feedback_fault_ = TorqueSafetyFault::FEEDBACK_TIMEOUT;
    } else {
      unfiltered_rate_rad_s_ = (angle - angle_rad_) / dt;
      const double alpha = -std::expm1(-2.0 * 3.14159265358979323846 * config_.rate_filter_hz * dt);
      rate_rad_s_ += alpha * (unfiltered_rate_rad_s_ - rate_rad_s_);
      rate_ready_ = true;
      // The hard ceiling sees the unfiltered rate, so a low LPF cannot hide overspeed.
      if (!std::isfinite(unfiltered_rate_rad_s_) || !std::isfinite(rate_rad_s_)) {
        feedback_fault_ = TorqueSafetyFault::INVALID_INPUT;
      } else if (std::abs(unfiltered_rate_rad_s_) > config_.hard_rate_limit_rad_s ||
        std::abs(rate_rad_s_) > config_.hard_rate_limit_rad_s)
      {
        feedback_fault_ = TorqueSafetyFault::RATE_LIMIT;
      }
    }
  }
  have_sample_ = true;
  angle_rad_ = angle;
  sample_time_sec_ = sample_time;
  return feedback_fault_;
}

void TorqueSteeringController::accountOutput(double now)
{
  if (have_output_time_) {
    const double dt = std::max(0.0, now - output_time_sec_);
    if (std::abs(last_command_raw_) > config_.sustained_limit_raw) peak_used_sec_ += dt;
    if (std::abs(last_command_raw_) <= config_.sustained_limit_raw * 0.5) {
      recovery_sec_ += dt;
      if (recovery_sec_ >= config_.peak_recovery_sec) peak_used_sec_ = 0.0;
    } else {
      recovery_sec_ = 0.0;
    }
  }
  output_time_sec_ = now;
  have_output_time_ = true;
}

TorqueSteeringOutput TorqueSteeringController::update(double target, double target_rate,
  double target_accel, double vehicle_speed, double now)
{
  TorqueSteeringOutput out;
  out.rate_rad_s = rate_rad_s_;
  out.unfiltered_rate_rad_s = unfiltered_rate_rad_s_;
  out.rate_ready = rate_ready_;
  out.fault = feedback_fault_;
  if (!std::isfinite(target) || !std::isfinite(target_rate) || !std::isfinite(now) ||
    !std::isfinite(target_accel) || !std::isfinite(vehicle_speed) ||
    (have_output_time_ && now < output_time_sec_))
  {
    out.fault = TorqueSafetyFault::INVALID_INPUT;
  } else if (!have_sample_ || now < sample_time_sec_ ||
    now - sample_time_sec_ > config_.feedback_timeout_sec)
  {
    out.fault = TorqueSafetyFault::FEEDBACK_TIMEOUT;
  }
  if (out.fault != TorqueSafetyFault::NONE) {
    idle(now);
    return out;
  }
  accountOutput(now);
  out.target_rad = std::clamp(target,
    config_.min_angle_rad + config_.soft_limit_margin_rad,
    config_.max_angle_rad - config_.soft_limit_margin_rad);
  out.error_rad = out.target_rad - angle_rad_;
  out.p_raw = config_.peak_limit_raw / config_.full_scale_error_rad * out.error_rad;
  out.d_raw = config_.kd * (target_rate - rate_rad_s_);
  out.target_accel_rad_s2 = target_accel;
  out.ff_raw = std::clamp(config_.inertia_ff * target_accel,
    -config_.feedforward_limit_raw, config_.feedforward_limit_raw);
  out.friction_raw = std::abs(out.error_rad) > config_.friction_deadband_rad ?
    std::copysign(config_.frictionRawAt(vehicle_speed), out.error_rad) : 0.0;
  out.unsaturated_raw = out.ff_raw + out.p_raw + out.d_raw + out.friction_raw;
  if (!std::isfinite(out.unsaturated_raw)) {
    out.fault = TorqueSafetyFault::INVALID_INPUT;
    last_command_raw_ = 0;
    return out;
  }
  out.peak_limited = peak_used_sec_ + config_.control_period_sec >
    config_.peak_duration_sec + 1e-9;
  out.limit_raw = out.peak_limited ? config_.sustained_limit_raw : config_.peak_limit_raw;
  out.settled = std::abs(out.error_rad) < config_.settle_angle_rad &&
    std::abs(rate_rad_s_) < config_.settle_rate_rad_s &&
    std::abs(unfiltered_rate_rad_s_) < config_.settle_rate_rad_s &&
    std::abs(target_rate) < config_.settle_rate_rad_s;
  double command = std::clamp(out.unsaturated_raw,
    -static_cast<double>(out.limit_raw), static_cast<double>(out.limit_raw));
  // Taper outward effort across the soft margin; inward recovery/braking keeps full authority.
  const double distance = command > 0.0 ? config_.max_angle_rad - angle_rad_ :
    angle_rad_ - config_.min_angle_rad;
  const double factor = std::clamp(distance / config_.soft_limit_margin_rad, 0.0, 1.0);
  out.direction_blocked = factor == 0.0 && command != 0.0;
  command *= factor;
  out.saturated = !out.settled && rate_ready_ &&
    (std::abs(out.unsaturated_raw) >= out.limit_raw ||
    std::abs(command - out.unsaturated_raw) > 1e-9);
  out.command_raw = out.settled || !rate_ready_ ? 0 : static_cast<int32_t>(std::lround(command));
  last_command_raw_ = out.command_raw;
  return out;
}

void TorqueSteeringController::idle(double now)
{
  if (std::isfinite(now) && (!have_output_time_ || now >= output_time_sec_)) accountOutput(now);
  last_command_raw_ = 0;
}

void TorqueSteeringController::reset(double now)
{
  idle(now);
  have_sample_ = rate_ready_ = false;
  angle_rad_ = sample_time_sec_ = rate_rad_s_ = unfiltered_rate_rad_s_ = 0.0;
  feedback_fault_ = TorqueSafetyFault::NONE;
}

double TargetAccelerationEstimator::observe(
  double target, double stamp, double filter_hz, double max_gap_sec)
{
  if (!std::isfinite(target) || !std::isfinite(stamp)) {
    reset();
    return accel_;
  }
  const double dt = stamp - stamp_;
  if (samples_ > 0 && (!(dt > 0.0) || dt > max_gap_sec)) reset();
  if (samples_ > 0) {
    const double rate = (target - target_) / dt;
    if (samples_ >= 2) {
      const double raw_accel = (rate - rate_) / (0.5 * (dt + dt_));
      const double alpha = std::isfinite(filter_hz) && filter_hz > 0.0 ?
        -std::expm1(-2.0 * 3.14159265358979323846 * filter_hz * dt) : 1.0;
      accel_ += alpha * (raw_accel - accel_);
      if (!std::isfinite(accel_)) accel_ = 0.0;
    }
    rate_ = rate;
    dt_ = dt;
  }
  target_ = target;
  stamp_ = stamp;
  samples_ = std::min(samples_ + 1, 2);
  return accel_;
}

void TargetAccelerationEstimator::reset()
{
  samples_ = 0;
  target_ = stamp_ = dt_ = rate_ = accel_ = 0.0;
}

int32_t keyaTorqueCommand(int32_t canonical_command, int direction_sign)
{
  if ((direction_sign != -1 && direction_sign != 1) ||
    canonical_command < -10000 || canonical_command > 10000)
  {
    throw std::invalid_argument("invalid KEYA torque direction or raw command");
  }
  return canonical_command * direction_sign;
}
}  // namespace lateral_ctrl
