#include "lateral_ctrl/wheel_angle_tracking_controller.hpp"

#include <gtest/gtest.h>
#include <cmath>
#include <limits>

TEST(WheelAngleTrackingController, DisabledMeasuresErrorWithoutCorrection)
{
  lateral_ctrl::WheelAngleTrackingController controller(false, 1.0, 1.0, 1.0, 1.0, 0.5);
  EXPECT_DOUBLE_EQ(controller.update(3.0, 1.0, 0.1), 3.0);
  EXPECT_DOUBLE_EQ(controller.errorDeg(), 2.0);
  EXPECT_DOUBLE_EQ(controller.correctionDeg(), 0.0);
}
TEST(WheelAngleTrackingController, ComputesPAndClampsCorrection)
{
  lateral_ctrl::WheelAngleTrackingController controller(true, 0.25, 0.0, 0.0, 1.0, 0.5);
  EXPECT_DOUBLE_EQ(controller.update(4.0, 2.0, 0.1), 4.5);
}
TEST(WheelAngleTrackingController, ClampsIntegralAndResets)
{
  lateral_ctrl::WheelAngleTrackingController controller(true, 0.0, 1.0, 0.0, 0.2, 10.0);
  controller.update(1.0, 0.0, 1.0);
  EXPECT_DOUBLE_EQ(controller.integralValue(), 0.2);
  controller.reset();
  EXPECT_DOUBLE_EQ(controller.integralValue(), 0.0);
}
TEST(WheelAngleTrackingController, InvalidDtSkipsIntegralAndDerivative)
{
  lateral_ctrl::WheelAngleTrackingController controller(true, 1.0, 1.0, 1.0, 1.0, 10.0);
  EXPECT_DOUBLE_EQ(controller.update(1.0, 0.0, -1.0), 2.0);
  EXPECT_DOUBLE_EQ(controller.integralValue(), 0.0);
}
TEST(WheelAngleTrackingController, RejectsNonFiniteAngles)
{
  lateral_ctrl::WheelAngleTrackingController controller(true, 1.0, 1.0, 1.0, 1.0, 10.0);
  EXPECT_THROW(
    controller.update(std::numeric_limits<double>::quiet_NaN(), 0.0, 0.1),
    std::invalid_argument);
  EXPECT_THROW(
    controller.update(0.0, std::numeric_limits<double>::quiet_NaN(), 0.1),
    std::invalid_argument);
}
TEST(WheelAngleTrackingController, RejectsNonFiniteConfiguration)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double infinity = std::numeric_limits<double>::infinity();
  EXPECT_THROW(
    lateral_ctrl::WheelAngleTrackingController(true, nan, 0.0, 0.0, 1.0, 0.5),
    std::invalid_argument);
  EXPECT_THROW(
    lateral_ctrl::WheelAngleTrackingController(true, 0.0, 0.0, 0.0, nan, 0.5),
    std::invalid_argument);
  EXPECT_THROW(
    lateral_ctrl::WheelAngleTrackingController(true, 0.0, 0.0, 0.0, 1.0, infinity),
    std::invalid_argument);
}
