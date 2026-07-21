#pragma once

#include <vector>

namespace lateral_ctrl
{
inline std::vector<double> defaultBenchSequence()
{
  return {0.0, 1.0, 0.0, -1.0, 0.0};
}
}  // namespace lateral_ctrl
