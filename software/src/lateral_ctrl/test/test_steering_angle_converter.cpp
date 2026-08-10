#include "lateral_ctrl/steering_angle_converter.hpp"

#include <gtest/gtest.h>

namespace
{
lateral_ctrl::SteeringAngleConverter MakeConverter()
{
  return lateral_ctrl::SteeringAngleConverter(
    3500.0, 46.79149152, -39.22366242, 22.73814759, 3321, 10000.0);
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
  EXPECT_NEAR(converter.countToEquivalentDeg(500), 6.5714969851, 1e-9);
  EXPECT_NEAR(converter.countToEquivalentDeg(3321), 28.3790454907, 1e-9);
  EXPECT_NEAR(converter.countToEquivalentDeg(-3321), -28.3790454907, 1e-9);
}
TEST(SteeringAngleConverter, DoesNotClampFeedback)
{
  const auto converter = MakeConverter();
  EXPECT_GT(converter.countToEquivalentDeg(3500), converter.countToEquivalentDeg(3321));
}
TEST(SteeringAngleConverter, ClampsCommandsToMeasuredCountRange)
{
  const auto converter = MakeConverter();
  EXPECT_EQ(converter.equivalentDegToCount(90.0), 3321);
  EXPECT_EQ(converter.equivalentDegToCount(-90.0), -3321);
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
