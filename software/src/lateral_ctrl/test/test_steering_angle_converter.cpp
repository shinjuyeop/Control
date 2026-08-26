#include "lateral_ctrl/steering_angle_converter.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace
{
lateral_ctrl::SteeringAngleConverter MakeConverter()
{
  return lateral_ctrl::SteeringAngleConverter(
    3500.0, 45.36712598527852, -3.534573536433758,
    -44.4281013734926, 2.7876017898037944, 28.96220619772027,
    3279, 10000.0);
}
}

TEST(SteeringAngleConverter, EncoderResolution)
{
  const auto converter = MakeConverter();
  EXPECT_DOUBLE_EQ(converter.countToMotorDeg(10000), 360.0);
  EXPECT_DOUBLE_EQ(converter.countToMotorDeg(1), 0.036);
}
TEST(SteeringAngleConverter, UsesMeasuredCanonicalRelation)
{
  const auto converter = MakeConverter();
  EXPECT_DOUBLE_EQ(converter.countToEquivalentDeg(0), 0.0);
  EXPECT_NEAR(converter.countToEquivalentDeg(500), 6.2822400916, 1e-9);
  EXPECT_NEAR(converter.countToEquivalentDeg(3279), 25.9178130178, 1e-9);
  EXPECT_NEAR(converter.countToEquivalentDeg(-3279), -27.8275011334, 1e-9);
}
TEST(SteeringAngleConverter, DoesNotClampFeedback)
{
  const auto converter = MakeConverter();
  EXPECT_GT(converter.countToEquivalentDeg(3500), converter.countToEquivalentDeg(3279));
}
TEST(SteeringAngleConverter, ClampsCommandsToMeasuredCountRange)
{
  const auto converter = MakeConverter();
  EXPECT_EQ(converter.equivalentDegToCount(90.0), 3279);
  EXPECT_EQ(converter.equivalentDegToCount(-90.0), -3279);
}
TEST(SteeringAngleConverter, RejectsNonFiniteCommand)
{
  const auto converter = MakeConverter();
  EXPECT_THROW(
    converter.equivalentDegToCount(std::numeric_limits<double>::quiet_NaN()),
    std::invalid_argument);
}
TEST(SteeringAngleConverter, RoundTripsEveryCommandCount)
{
  const auto converter = MakeConverter();
  for (int32_t count = -converter.commandCountLimit();
    count <= converter.commandCountLimit(); ++count)
  {
    EXPECT_EQ(converter.equivalentDegToCount(converter.countToEquivalentDeg(count)), count);
  }
}
