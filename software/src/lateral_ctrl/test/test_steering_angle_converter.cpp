#include "lateral_ctrl/steering_angle_converter.hpp"

#include <gtest/gtest.h>

TEST(SteeringAngleConverter, EncoderResolution)
{
  lateral_ctrl::SteeringAngleConverter converter;
  EXPECT_EQ(converter.motorDegToCount(360.0), 10000);
  EXPECT_EQ(converter.motorDegToCount(0.036), 1);
}
TEST(SteeringAngleConverter, ClampsWheelAngle)
{
  lateral_ctrl::SteeringAngleConverter converter;
  EXPECT_DOUBLE_EQ(converter.clampWheelDeg(40.0), 30.0);
  EXPECT_DOUBLE_EQ(converter.clampWheelDeg(-40.0), -30.0);
}
TEST(SteeringAngleConverter, RoundTripsCalibratedAngle)
{
  lateral_ctrl::SteeringAngleConverter converter;
  for (double wheel : {-25.0, -5.0, -1.0, 1.0, 5.0, 25.0}) {
    EXPECT_NEAR(converter.motorToWheelDeg(converter.wheelToMotorDeg(wheel)), wheel, 1e-6);
  }
}
