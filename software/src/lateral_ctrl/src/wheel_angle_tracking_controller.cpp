#include "lateral_ctrl/wheel_angle_tracking_controller.hpp"

#include <algorithm>
#include <cmath>

namespace lateral_ctrl
{
WheelAngleTrackingController::WheelAngleTrackingController(
  bool enabled, double kp, double ki, double kd, double integral_limit,
  double correction_limit_deg)
: enabled_(enabled), kp_(kp), ki_(ki), kd_(kd),
  integral_limit_(std::abs(integral_limit)),
  correction_limit_deg_(std::abs(correction_limit_deg)) {}

double WheelAngleTrackingController::update(double target, double actual, double dt)
{
  if (!std::isfinite(target) || !std::isfinite(actual)) {
    reset();
    return std::isfinite(target) ? target : 0.0;
  }
  error_deg_ = target - actual;
  if (!enabled_) {
    correction_deg_ = 0.0;
    integral_ = 0.0;
    have_previous_ = false;
    return target;
  }
  double derivative = 0.0;
  if (std::isfinite(dt) && dt > 0.0) {
    integral_ = std::clamp(integral_ + error_deg_ * dt, -integral_limit_, integral_limit_);
    if (have_previous_) derivative = (error_deg_ - previous_error_) / dt;
  }
  correction_deg_ = std::clamp(
    kp_ * error_deg_ + ki_ * integral_ + kd_ * derivative,
    -correction_limit_deg_, correction_limit_deg_);
  previous_error_ = error_deg_;
  have_previous_ = true;
  return target + correction_deg_;
}

void WheelAngleTrackingController::reset()
{
  have_previous_ = false;
  error_deg_ = previous_error_ = correction_deg_ = integral_ = 0.0;
}
void WheelAngleTrackingController::setEnabled(bool enabled)
{
  if (enabled_ != enabled) reset();
  enabled_ = enabled;
}
}  // namespace lateral_ctrl
