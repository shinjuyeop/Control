#include "control_tools/bench_test_utils.hpp"
#include <gtest/gtest.h>
TEST(SteeringSafetyLogic, BenchSequenceAlternatesFiveDegrees)
{
  EXPECT_EQ(
    control_tools::defaultBenchSequence(),
    (std::vector<double>{5.0, -5.0}));
}
