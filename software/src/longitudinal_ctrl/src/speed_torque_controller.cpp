#include "longitudinal_ctrl/speed_torque_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace longitudinal_ctrl
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
double clamp(double value, double low, double high)
{
  return std::min(std::max(value, low), high);
}
bool positive(double value) {return std::isfinite(value) && value > 0.0;}
bool nonnegative(double value) {return std::isfinite(value) && value >= 0.0;}
}  // namespace

void SpeedTorqueConfig::validate() const
{
  if (!std::isfinite(control_period_sec) || control_period_sec < 0.005 ||
    control_period_sec > 0.1)
  {
    throw std::invalid_argument("control_period_sec must be within 5..100 ms");
  }
  if (!positive(current_gain_mps2_per_a)) {
    throw std::invalid_argument("current_gain_mps2_per_a must be finite and positive");
  }
  for (double value : {rolling_resistance_mps2, speed_damping_per_s, feedforward_accel_scale,
      feedforward_resistance_scale, kp_a_per_mps, ki_a_per_m, integrator_limit_a,
      max_regen_current_a, regen_min_speed_mps, stop_speed_mps})
  {
    if (!nonnegative(value)) {
      throw std::invalid_argument("plant, gain, limit and speed thresholds must be finite and >= 0");
    }
  }
  for (double value : {accel_limit_mps2, decel_limit_mps2, tracking_error_limit_mps,
      max_drive_current_a, regen_bus_current_limit_a, current_slew_a_per_s,
      speed_filter_hz, speed_jump_reject_mps})
  {
    if (!positive(value)) {
      throw std::invalid_argument("ramp limits, current envelope and speed filter must be finite and positive");
    }
  }
  if (speed_filter_hz > 0.5 / control_period_sec) {
    throw std::invalid_argument("speed_filter_hz exceeds half the control rate");
  }
  if (speed_jump_reject_max_samples < 1) {
    throw std::invalid_argument("speed_jump_reject_max_samples must be >= 1");
  }
  if (!std::isfinite(feedback_timeout_sec) || feedback_timeout_sec < 2.0 * control_period_sec ||
    feedback_timeout_sec > 2.0)
  {
    throw std::invalid_argument("feedback_timeout_sec must be within 2 control periods .. 2 s");
  }
}

SpeedTorqueController::SpeedTorqueController(const SpeedTorqueConfig & config)
: config_(config)
{
  config_.validate();
  regen_cap_a_ = config_.max_regen_current_a;
}

void SpeedTorqueController::configure(const SpeedTorqueConfig & config)
{
  config.validate();
  config_ = config;
  regen_cap_a_ = std::min(regen_cap_a_, config_.max_regen_current_a);
  if (!active_) regen_cap_a_ = config_.max_regen_current_a;
  integrator_a_ = clamp(integrator_a_, -config_.integrator_limit_a, config_.integrator_limit_a);
}

void SpeedTorqueController::resetSpeedEstimate()
{
  have_speed_ = false;
  speed_mps_ = 0.0;
  raw_speed_mps_ = 0.0;
  rejected_streak_ = 0;
}

bool SpeedTorqueController::observeSpeed(double speed_mps, double time_sec)
{
  if (!std::isfinite(speed_mps) || !std::isfinite(time_sec)) return false;
  if (!have_speed_ || time_sec < last_sample_time_sec_) {
    // 첫 샘플이거나 시각이 역행하면 추정을 새로 시작한다.
    have_speed_ = true;
    speed_mps_ = speed_mps;
    raw_speed_mps_ = speed_mps;
    last_sample_time_sec_ = time_sec;
    rejected_streak_ = 0;
    return true;
  }
  raw_speed_mps_ = speed_mps;
  const double jump = std::fabs(speed_mps - speed_mps_);
  if (jump > config_.speed_jump_reject_mps &&
    rejected_streak_ < config_.speed_jump_reject_max_samples)
  {
    // spike는 버리고 추정을 유지한다. 샘플 시각도 갱신하지 않아 연속되면
    // feedback timeout으로 드러난다. 연속 거부가 상한에 닿으면 다시 받아들인다.
    ++rejected_streak_;
    ++rejected_total_;
    return false;
  }
  const double dt = std::max(time_sec - last_sample_time_sec_, 1e-4);
  const double tau = 1.0 / (2.0 * kPi * config_.speed_filter_hz);
  const double alpha = (rejected_streak_ >= config_.speed_jump_reject_max_samples) ?
    1.0 : dt / (tau + dt);
  speed_mps_ += alpha * (speed_mps - speed_mps_);
  last_sample_time_sec_ = time_sec;
  rejected_streak_ = 0;
  return true;
}

void SpeedTorqueController::observeBusCurrent(double bus_current_a)
{
  if (std::isfinite(bus_current_a)) bus_current_a_ = bus_current_a;
}

bool SpeedTorqueController::feedbackFresh(double now_sec) const
{
  return have_speed_ && std::isfinite(now_sec) &&
         (now_sec - last_sample_time_sec_) <= config_.feedback_timeout_sec;
}

double SpeedTorqueController::resistanceCurrentA(double speed_mps) const
{
  return (config_.rolling_resistance_mps2 + config_.speed_damping_per_s * speed_mps) /
         config_.current_gain_mps2_per_a;
}

void SpeedTorqueController::start(double now_sec, double initial_current_a)
{
  active_ = true;
  last_update_time_sec_ = now_sec;
  shaped_target_mps_ = std::max(0.0, speed_mps_);
  filtered_target_mps_ = shaped_target_mps_;
  regen_cap_a_ = config_.max_regen_current_a;
  if (!std::isfinite(initial_current_a)) initial_current_a = 0.0;
  // 첫 출력이 실측 전류와 같아지도록 적분기를 맞춘다(EZ speed control에서 bumpless 전환).
  const double ff0 = config_.feedforward_resistance_scale * resistanceCurrentA(shaped_target_mps_);
  integrator_a_ = clamp(initial_current_a - ff0,
      -config_.integrator_limit_a, config_.integrator_limit_a);
  last_command_a_ = clamp(initial_current_a,
      -config_.max_regen_current_a, config_.max_drive_current_a);
}

void SpeedTorqueController::stop()
{
  active_ = false;
  integrator_a_ = 0.0;
  last_command_a_ = 0.0;
  shaped_target_mps_ = 0.0;
  filtered_target_mps_ = 0.0;
  regen_cap_a_ = config_.max_regen_current_a;
}

SpeedTorqueOutput SpeedTorqueController::update(double target_mps, double now_sec)
{
  SpeedTorqueOutput out;
  if (!active_) start(now_sec, 0.0);
  if (!std::isfinite(target_mps)) target_mps = 0.0;
  target_mps = std::max(0.0, target_mps);
  const double period = config_.control_period_sec;
  const double dt = clamp(now_sec - last_update_time_sec_, 0.25 * period, 5.0 * period);
  last_update_time_sec_ = now_sec;

  const double v = speed_mps_;
  out.target_mps = target_mps;
  out.speed_mps = v;

  // 목표 형상: 가감속 한계 → 실측 차속 주변으로 제한 → 0 이상.
  const double previous = shaped_target_mps_;
  double shaped = clamp(target_mps, previous - config_.decel_limit_mps2 * dt,
      previous + config_.accel_limit_mps2 * dt);
  shaped = clamp(shaped, v - config_.tracking_error_limit_mps, v + config_.tracking_error_limit_mps);
  shaped = std::max(0.0, shaped);
  const double accel_bound = std::max(config_.accel_limit_mps2, config_.decel_limit_mps2);
  const double target_accel = clamp((shaped - previous) / dt, -accel_bound, accel_bound);
  shaped_target_mps_ = shaped;
  out.shaped_target_mps = shaped;
  out.target_accel_mps2 = target_accel;

  // 실측 차속과 같은 1차 필터를 목표에도 걸어 지연을 맞춘다.
  const double tau = 1.0 / (2.0 * kPi * config_.speed_filter_hz);
  filtered_target_mps_ += dt / (tau + dt) * (shaped - filtered_target_mps_);
  out.filtered_target_mps = filtered_target_mps_;

  const double error = filtered_target_mps_ - v;
  out.error_mps = error;
  out.ff_a = config_.feedforward_accel_scale * target_accel / config_.current_gain_mps2_per_a +
    config_.feedforward_resistance_scale * resistanceCurrentA(shaped);
  out.p_a = config_.kp_a_per_mps * error;

  const double upper = config_.max_drive_current_a;
  out.regen_blocked = v < config_.regen_min_speed_mps;
  const double lower = out.regen_blocked ? 0.0 : -regen_cap_a_;
  out.stopped = target_mps <= config_.stop_speed_mps && v <= config_.stop_speed_mps;

  double command = 0.0;
  if (out.stopped) {
    integrator_a_ = 0.0;
    shaped_target_mps_ = 0.0;
    filtered_target_mps_ = 0.0;
    out.shaped_target_mps = 0.0;
    out.filtered_target_mps = 0.0;
    out.unsaturated_a = 0.0;
  } else {
    double unsaturated = out.ff_a + out.p_a + integrator_a_;
    const bool saturated_high = unsaturated > upper;
    const bool saturated_low = unsaturated < lower;
    // 조건부 적분: 포화 방향으로 더 쌓지 않는다.
    if (!(saturated_high && error > 0.0) && !(saturated_low && error < 0.0)) {
      integrator_a_ = clamp(integrator_a_ + config_.ki_a_per_m * error * dt,
          -config_.integrator_limit_a, config_.integrator_limit_a);
    }
    unsaturated = out.ff_a + out.p_a + integrator_a_;
    out.unsaturated_a = unsaturated;
    command = clamp(unsaturated, lower, upper);
    out.saturated = command != unsaturated;
  }
  out.i_a = integrator_a_;

  const double max_step = config_.current_slew_a_per_s * dt;
  const double slewed = clamp(command, last_command_a_ - max_step, last_command_a_ + max_step);
  out.slew_limited = slewed != command;
  command = slewed;
  last_command_a_ = command;
  out.command_a = command;

  // DC bus 충전전류 감시: 회생 중 한계를 넘으면 회생 상한을 줄이고, 아니면 천천히 되돌린다.
  const double regen_floor = std::min(10.0, config_.max_regen_current_a);
  if (command < 0.0 && bus_current_a_ < -config_.regen_bus_current_limit_a) {
    regen_cap_a_ = std::max(regen_floor, regen_cap_a_ * 0.95);
  } else {
    regen_cap_a_ = std::min(config_.max_regen_current_a,
        regen_cap_a_ + 0.01 * config_.max_regen_current_a);
  }
  out.regen_cap_a = regen_cap_a_;
  return out;
}
}  // namespace longitudinal_ctrl
