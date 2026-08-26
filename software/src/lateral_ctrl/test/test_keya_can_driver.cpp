#include "lateral_ctrl/keya_can_driver.hpp"
#include "keya_can_frame_decoder.hpp"

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
  lateral_ctrl::KeyaMotorState state;
  const std::array<uint8_t, 8> payload{
    0x60, 0x04, 0x21, 0x02, 0x00, 0x00, 0x27, 0x10};
  const auto received_at =
    std::chrono::steady_clock::time_point(std::chrono::milliseconds(42));

  EXPECT_TRUE(lateral_ctrl::detail::applyKeyaCanFrame(
      state, 0x05800001, 0x07000001, 0x05800001, 8, payload.data(), received_at));
  EXPECT_TRUE(state.position_valid);
  EXPECT_EQ(state.motor_position_count, 10000);
  EXPECT_EQ(state.last_position_feedback, received_at);
  EXPECT_FALSE(state.enabled_feedback_valid);
}
