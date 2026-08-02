#include "lateral_ctrl/steering_angle_converter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lateral_ctrl
{
SteeringAngleConverter::SteeringAngleConverter(
  double a, double b, double c, double counts_per_rev, double direction_sign,
  double wheel_limit_deg)
: a_(a), b_(b), c_(c), counts_per_rev_(counts_per_rev),
  direction_sign_(std::abs(direction_sign) > 1e-12 ? std::copysign(1.0, direction_sign) : 1.0),
  wheel_limit_deg_(std::abs(wheel_limit_deg)) {}

double SteeringAngleConverter::clampWheelDeg(double value) const
{
  if (!std::isfinite(value)) return 0.0;
  return std::clamp(value, -wheel_limit_deg_, wheel_limit_deg_);
}

double SteeringAngleConverter::wheelToMotorDeg(double wheel_deg) const
{
  const double y = clampWheelDeg(wheel_deg);
  const double magnitude = std::abs(y);
  double motor = 0.0;
  if (magnitude > 0.0) {
    if (std::abs(a_) < 1e-12) {
      motor = std::max(0.0, (magnitude - c_) / b_);
    } else {
      const double discriminant = b_ * b_ - 4.0 * a_ * (c_ - magnitude);
      motor = std::max(0.0, (-b_ + std::sqrt(std::max(0.0, discriminant))) / (2.0 * a_));
    }
  }
  return direction_sign_ * std::copysign(motor, y);
}

double SteeringAngleConverter::motorToWheelDeg(double motor_deg) const
{
  if (!std::isfinite(motor_deg)) return 0.0;
  const double normalized = motor_deg / direction_sign_;
  if (std::abs(normalized) < 1e-12) return 0.0;
  const double magnitude = std::abs(normalized);
  // 측정 경로에는 wheel_limit_deg를 걸지 않는다. 그 값은 명령을 제한하는 한계이지
  // 바퀴가 실제로 놓일 수 있는 위치의 한계가 아니다. 여기에 상한을 씌우면 한계를
  // 넘은 실제 각도가 한계값으로 보고되어 추종 오차가 실제보다 작게 나온다.
  return std::copysign(a_ * magnitude * magnitude + b_ * magnitude + c_, normalized);
}

int32_t SteeringAngleConverter::motorDegToCount(double motor_deg) const
{
  if (!std::isfinite(motor_deg) || counts_per_rev_ <= 0.0) return 0;
  const double value = std::round(motor_deg * counts_per_rev_ / 360.0);
  return static_cast<int32_t>(std::clamp(
    value, static_cast<double>(std::numeric_limits<int32_t>::min()),
    static_cast<double>(std::numeric_limits<int32_t>::max())));
}

double SteeringAngleConverter::countToMotorDeg(int32_t count) const
{
  return counts_per_rev_ > 0.0 ? static_cast<double>(count) * 360.0 / counts_per_rev_ : 0.0;
}
}  // namespace lateral_ctrl
