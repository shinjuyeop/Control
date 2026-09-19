#include "lateral_ctrl/steering_angle_converter.hpp"

#include <gtest/gtest.h>

#include <limits>

namespace
{
lateral_ctrl::SteeringAngleConverter MakeConverter()
{
  return lateral_ctrl::SteeringAngleConverter(
    3500.0, 20.801783967214856, -0.016893521554765867,
    -7.409984217512384, -0.02066649605375595, 2.8146092208162923,
    5300, 10000.0);
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
  EXPECT_NEAR(converter.countToEquivalentDeg(500), 2.94989, 1e-5);
  EXPECT_NEAR(converter.countToEquivalentDeg(5300), 28.033175, 1e-5);
  EXPECT_NEAR(converter.countToEquivalentDeg(-5300), -28.327985, 1e-5);
}
TEST(SteeringAngleConverter, DoesNotClampFeedback)
{
  const auto converter = MakeConverter();
  EXPECT_GT(converter.countToEquivalentDeg(5301), converter.countToEquivalentDeg(5300));
}
TEST(SteeringAngleConverter, ConvertsRequestedAnglesToWheelMotorCounts)
{
  const auto converter = MakeConverter();
  EXPECT_EQ(converter.equivalentDegToCount(5.0), 859);
  EXPECT_EQ(converter.equivalentDegToCount(10.0), 1848);
  EXPECT_EQ(converter.equivalentDegToCount(-10.0), -1845);
  EXPECT_EQ(converter.equivalentDegToCount(25.0), 5027);
  EXPECT_EQ(converter.equivalentDegToCount(-25.0), -5002);
}
TEST(SteeringAngleConverter, ClampsCommandsToConfiguredCountRange)
{
  const auto converter = MakeConverter();
  EXPECT_EQ(converter.equivalentDegToCount(90.0), 5300);
  EXPECT_EQ(converter.equivalentDegToCount(-90.0), -5300);
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
