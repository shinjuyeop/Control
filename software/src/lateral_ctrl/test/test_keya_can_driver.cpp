#include "lateral_ctrl/keya_can_driver.hpp"
#include "keya_can_frame_decoder.hpp"

#include <gtest/gtest.h>
#include <linux/can.h>
#include <sys/socket.h>
#include <unistd.h>
#include <thread>
#include <cstring>

namespace lateral_ctrl
{
class KeyaCanDriverTestPeer
{
public:
  static void attachSocket(KeyaCanDriver & driver, int fd) { driver.socket_fd_ = fd; }
};
}

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

TEST(KeyaCanDriver, EncodesPositiveAndNegativeTorqueBigEndian)
{
  EXPECT_EQ(lateral_ctrl::KeyaCanDriver::torquePayload(10000),
    (std::array<uint8_t, 8>{0x23, 0x01, 0x20, 0x01, 0, 0, 0x27, 0x10}));
  EXPECT_EQ(lateral_ctrl::KeyaCanDriver::torquePayload(-10000),
    (std::array<uint8_t, 8>{0x23, 0x01, 0x20, 0x01, 0xFF, 0xFF, 0xD8, 0xF0}));
  EXPECT_EQ(lateral_ctrl::KeyaCanDriver::torquePayload(-8000),
    (std::array<uint8_t, 8>{0x23, 0x01, 0x20, 0x01, 0xFF, 0xFF, 0xE0, 0xC0}));
  EXPECT_EQ(lateral_ctrl::KeyaCanDriver::torquePayload(0),
    (std::array<uint8_t, 8>{0x23, 0x01, 0x20, 0x01, 0, 0, 0, 0}));
  EXPECT_THROW(lateral_ctrl::KeyaCanDriver::torquePayload(10001), std::out_of_range);
  EXPECT_THROW(lateral_ctrl::KeyaCanDriver::torquePayload(-10001), std::out_of_range);
}

TEST(KeyaCanDriver, HeartbeatCannotRefreshAuthoritativePosition)
{
  lateral_ctrl::KeyaMotorState state;
  const std::array<uint8_t, 8> heartbeat{0, 3, 0, 10, 0, 1, 0, 0};
  const auto received = std::chrono::steady_clock::now();
  EXPECT_TRUE(lateral_ctrl::detail::applyKeyaCanFrame(
    state, 0x05800001, 0x07000001, 0x07000001, 8, heartbeat.data(), received));
  EXPECT_EQ(state.last_feedback, received);
  EXPECT_EQ(state.last_position_feedback, std::chrono::steady_clock::time_point{});
  EXPECT_FALSE(state.position_valid);
}

TEST(KeyaCanDriver, KernelArrivalTimestampPreventsQueuedFeedbackLookingFresh)
{
  int sockets[2];
  ASSERT_EQ(socketpair(AF_UNIX, SOCK_DGRAM | SOCK_NONBLOCK, 0, sockets), 0);
  lateral_ctrl::KeyaCanDriver driver("unused", 1);
  lateral_ctrl::KeyaCanDriverTestPeer::attachSocket(driver, sockets[1]);
  const int enabled = 1;
  ASSERT_EQ(setsockopt(sockets[1], SOL_SOCKET, SO_TIMESTAMPNS, &enabled, sizeof(enabled)), 0);
  can_frame frame{};
  frame.can_id = CAN_EFF_FLAG | 0x05800001;
  frame.can_dlc = 8;
  const std::array<uint8_t, 8> position{0x60, 0x04, 0x21, 0x02, 0, 0, 0, 77};
  std::memcpy(frame.data, position.data(), position.size());
  ASSERT_EQ(write(sockets[0], &frame, sizeof(frame)), static_cast<ssize_t>(sizeof(frame)));
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  driver.receiveAvailableFrames();
  ASSERT_TRUE(driver.motorState().position_valid);
  EXPECT_EQ(driver.motorState().motor_position_count, 77);
  EXPECT_GE(std::chrono::steady_clock::now() - driver.motorState().last_position_feedback,
    std::chrono::milliseconds(15));
  const auto stamp = driver.motorState().last_position_feedback;
  // RTR/error frames must never refresh the valid position or communication timestamp.
  frame.can_id |= CAN_RTR_FLAG;
  ASSERT_EQ(write(sockets[0], &frame, sizeof(frame)), static_cast<ssize_t>(sizeof(frame)));
  driver.receiveAvailableFrames();
  EXPECT_EQ(driver.motorState().last_position_feedback, stamp);
  close(sockets[0]);
}
