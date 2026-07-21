#include "lateral_ctrl/keya_can_driver.hpp"

#include <gtest/gtest.h>

TEST(KeyaCanDriver, BuildsEnableFrame)
{
  EXPECT_EQ(
    lateral_ctrl::KeyaCanDriver::enablePayload(),
    (std::array<uint8_t, 8>{0x23, 0x0D, 0x20, 0x01, 0, 0, 0, 0}));
}
TEST(KeyaCanDriver, BuildsCorrectDisableFrame)
{
  EXPECT_EQ(
    lateral_ctrl::KeyaCanDriver::disablePayload(),
    (std::array<uint8_t, 8>{0x23, 0x0C, 0x20, 0x01, 0, 0, 0, 0}));
}
TEST(KeyaCanDriver, EncodesSignedPositionBigEndian)
{
  EXPECT_EQ(
    lateral_ctrl::KeyaCanDriver::positionPayload(-1217),
    (std::array<uint8_t, 8>{0x23, 0x02, 0x20, 0x01, 0xFF, 0xFF, 0xFB, 0x3F}));
}
TEST(KeyaCanDriver, ParsesAuthoritativeEncoderQuery)
{
  lateral_ctrl::KeyaCanDriver driver("vcan0", 1);
  driver.parseFrameForTest(
    0x05800001, {0x60, 0x04, 0x21, 0x02, 0x00, 0x00, 0x27, 0x10});
  EXPECT_TRUE(driver.motorState().position_valid);
  EXPECT_EQ(driver.motorState().motor_position_count, 10000);
  EXPECT_FALSE(driver.motorState().enabled_feedback_valid);
}
