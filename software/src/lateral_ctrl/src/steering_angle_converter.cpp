#include "lateral_ctrl/steering_angle_converter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace lateral_ctrl
{
SteeringAngleConverter::SteeringAngleConverter(
  double count_scale, double equivalent_k1_deg, double equivalent_k2_deg,
  double equivalent_k3_deg, double equivalent_k4_deg, double equivalent_k5_deg,
  std::int64_t command_count_limit, double counts_per_rev)
: count_scale_(count_scale), equivalent_k1_deg_(equivalent_k1_deg),
  equivalent_k2_deg_(equivalent_k2_deg), equivalent_k3_deg_(equivalent_k3_deg),
  equivalent_k4_deg_(equivalent_k4_deg), equivalent_k5_deg_(equivalent_k5_deg),
  command_count_limit_(0), counts_per_rev_(counts_per_rev),
  command_min_angle_deg_(0.0), command_max_angle_deg_(0.0)
{
  if (!std::isfinite(count_scale_) || count_scale_ <= 0.0 ||
    !std::isfinite(equivalent_k1_deg_) || !std::isfinite(equivalent_k2_deg_) ||
    !std::isfinite(equivalent_k3_deg_) || !std::isfinite(equivalent_k4_deg_) ||
    !std::isfinite(equivalent_k5_deg_) || command_count_limit <= 0 ||
    command_count_limit > std::numeric_limits<int32_t>::max() ||
    !std::isfinite(counts_per_rev_) || counts_per_rev_ <= 0.0)
  {
    throw std::invalid_argument("invalid steering calibration parameters");
  }
  command_count_limit_ = static_cast<int32_t>(command_count_limit);
  command_min_angle_deg_ = countToEquivalentDeg(-command_count_limit_);
  command_max_angle_deg_ = countToEquivalentDeg(command_count_limit_);
  double previous = countToEquivalentDeg(-command_count_limit_);
  for (int32_t count = -command_count_limit_ + 1; count <= command_count_limit_; ++count) {
    const double current = countToEquivalentDeg(count);
    if (!(current > previous)) {
      throw std::invalid_argument("steering calibration must increase over the command range");
    }
    previous = current;
  }
}

double SteeringAngleConverter::clampEquivalentDeg(double value) const
{
  if (!std::isfinite(value)) return 0.0;
  return std::clamp(value, command_min_angle_deg_, command_max_angle_deg_);
}

double SteeringAngleConverter::countToEquivalentDeg(int32_t count) const
{
  const double v = static_cast<double>(count) / count_scale_;
  return v * (equivalent_k1_deg_ + v *
    (equivalent_k2_deg_ + v *
    (equivalent_k3_deg_ + v *
    (equivalent_k4_deg_ + v * equivalent_k5_deg_))));
}

int32_t SteeringAngleConverter::equivalentDegToCount(double equivalent_deg) const
{
  const double target = clampEquivalentDeg(equivalent_deg);
  int32_t lower = -command_count_limit_;
  int32_t upper = command_count_limit_;
  while (lower < upper) {
    const int32_t middle = lower + (upper - lower) / 2;
    if (countToEquivalentDeg(middle) < target) lower = middle + 1;
    else upper = middle;
  }
  const int32_t upper_candidate = lower;
  const int32_t lower_candidate = std::max(-command_count_limit_, upper_candidate - 1);
  const double lower_error = std::abs(countToEquivalentDeg(lower_candidate) - target);
  const double upper_error = std::abs(countToEquivalentDeg(upper_candidate) - target);
  return lower_error <= upper_error ? lower_candidate : upper_candidate;
}

double SteeringAngleConverter::countToMotorDeg(int32_t count) const
{
  return counts_per_rev_ > 0.0 ? static_cast<double>(count) * 360.0 / counts_per_rev_ : 0.0;
}
}  // namespace lateral_ctrl
