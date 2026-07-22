#pragma once

#include <cstdint>

namespace lateral_ctrl
{
class SteeringAngleConverter
{
public:
  SteeringAngleConverter(
    double a = 0.0002, double b = 0.2159, double c = 0.1612,
    double counts_per_rev = 10000.0, double direction_sign = -1.0,
    double wheel_limit_deg = 30.0);
  double wheelToMotorDeg(double wheel_deg) const;
  double motorToWheelDeg(double motor_deg) const;
  int32_t motorDegToCount(double motor_deg) const;
  double countToMotorDeg(int32_t count) const;
  double clampWheelDeg(double wheel_deg) const;

private:
  double a_, b_, c_, counts_per_rev_, direction_sign_, wheel_limit_deg_;
};
}  // namespace lateral_ctrl
