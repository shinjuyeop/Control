#include "lateral_ctrl/steering_angle_converter.hpp"

#include <gtest/gtest.h>

#include <cmath>

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
TEST(SteeringAngleConverter, DoesNotClampMeasuredWheelAngle)
{
  lateral_ctrl::SteeringAngleConverter converter;
  // 한계를 넘어선 위치에 바퀴가 실제로 놓일 수 있다. 측정 경로는 그 값을 그대로
  // 돌려줘야 추종 오차가 실제 크기로 잡힌다.
  const double beyond_limit_motor_deg = converter.wheelToMotorDeg(30.0) * 1.5;
  EXPECT_GT(std::abs(converter.motorToWheelDeg(beyond_limit_motor_deg)), 30.0);
}
TEST(SteeringAngleConverter, ClampsCommandedWheelAngle)
{
  lateral_ctrl::SteeringAngleConverter converter;
  // 명령 경로는 반대다. 한계 밖을 지시해도 한계 안의 모터 각도로 접힌다.
  EXPECT_DOUBLE_EQ(converter.wheelToMotorDeg(40.0), converter.wheelToMotorDeg(30.0));
}
TEST(SteeringAngleConverter, RoundTripsCalibratedAngle)
{
  lateral_ctrl::SteeringAngleConverter converter;
  for (double wheel : {-25.0, -5.0, -1.0, 1.0, 5.0, 25.0}) {
    EXPECT_NEAR(converter.motorToWheelDeg(converter.wheelToMotorDeg(wheel)), wheel, 1e-6);
  }
}
