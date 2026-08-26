#pragma once

#include <cstdint>

namespace lateral_ctrl
{
class SteeringAngleConverter
{
public:
  SteeringAngleConverter(
    double count_scale, double equivalent_k1_deg, double equivalent_k2_deg,
    double equivalent_k3_deg, double equivalent_k4_deg, double equivalent_k5_deg,
    std::int64_t command_count_limit, double counts_per_rev);
  int32_t equivalentDegToCount(double equivalent_deg) const;
  double countToEquivalentDeg(int32_t count) const;
  double countToMotorDeg(int32_t count) const;
  double clampEquivalentDeg(double equivalent_deg) const;
  int32_t commandCountLimit() const { return command_count_limit_; }

private:
  double count_scale_;
  double equivalent_k1_deg_;
  double equivalent_k2_deg_;
  double equivalent_k3_deg_;
  double equivalent_k4_deg_;
  double equivalent_k5_deg_;
  int32_t command_count_limit_;
  double counts_per_rev_;
  double command_min_angle_deg_;
  double command_max_angle_deg_;
};
}  // namespace lateral_ctrl
