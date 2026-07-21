#include "lateral_ctrl/keya_can_driver.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <utility>

namespace lateral_ctrl
{
namespace
{
int16_t be16(const uint8_t * p) { return static_cast<int16_t>((p[0] << 8) | p[1]); }
int32_t be32(const uint8_t * p)
{
  return static_cast<int32_t>(
    (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
    (static_cast<uint32_t>(p[2]) << 8) | p[3]);
}
}

KeyaCanDriver::KeyaCanDriver(std::string interface_name, uint8_t node_id)
: interface_name_(std::move(interface_name)),
  command_id_(0x06000000U + node_id), feedback_id_(0x05800000U + node_id),
  heartbeat_id_(0x07000000U + node_id) {}
KeyaCanDriver::~KeyaCanDriver() { close(); }

bool KeyaCanDriver::open()
{
  close();
  socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) return false;
  const can_filter filters[] = {
    {feedback_id_ | CAN_EFF_FLAG, CAN_EFF_FLAG | CAN_EFF_MASK},
    {heartbeat_id_ | CAN_EFF_FLAG, CAN_EFF_FLAG | CAN_EFF_MASK}
  };
  if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, filters, sizeof(filters)) < 0) {
    close();
    return false;
  }
  const int flags = fcntl(socket_fd_, F_GETFL, 0);
  if (flags < 0 || fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    close(); return false;
  }
  ifreq ifr{};
  std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
  if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
    close();
    return false;
  }
  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = ifr.ifr_ifindex;
  if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
    close(); return false;
  }
  return true;
}
void KeyaCanDriver::close()
{
  if (socket_fd_ >= 0) ::close(socket_fd_);
  socket_fd_ = -1;
}
std::array<uint8_t, 8> KeyaCanDriver::enablePayload()
{
  return {0x23, 0x0D, 0x20, 0x01, 0, 0, 0, 0};
}
std::array<uint8_t, 8> KeyaCanDriver::disablePayload()
{
  return {0x23, 0x0C, 0x20, 0x01, 0, 0, 0, 0};
}
std::array<uint8_t, 8> KeyaCanDriver::positionPayload(int32_t count)
{
  const auto value = static_cast<uint32_t>(count);
  return {0x23, 0x02, 0x20, 0x01, static_cast<uint8_t>(value >> 24),
    static_cast<uint8_t>(value >> 16), static_cast<uint8_t>(value >> 8),
    static_cast<uint8_t>(value)};
}
std::array<uint8_t, 8> KeyaCanDriver::absolutePositionModePayload()
{
  return {0x03, 0x0D, 0x20, 0x31, 0, 0, 0, 0};
}
bool KeyaCanDriver::send(const std::array<uint8_t, 8> & data)
{
  if (socket_fd_ < 0) return false;
  can_frame frame{};
  frame.can_id = command_id_ | CAN_EFF_FLAG;
  frame.can_dlc = 8;
  std::memcpy(frame.data, data.data(), data.size());
  return write(socket_fd_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
}
bool KeyaCanDriver::sendEnable() { return send(enablePayload()); }
bool KeyaCanDriver::sendDisable() { return send(disablePayload()); }
bool KeyaCanDriver::sendPositionCount(int32_t count) { return send(positionPayload(count)); }
bool KeyaCanDriver::sendAbsolutePositionMode()
{
  const bool sent = send(absolutePositionModePayload());
  if (sent) state_.absolute_position_mode_commanded = true;
  return sent;
}
bool KeyaCanDriver::requestEncoderPosition()
{
  return send({0x40, 0x04, 0x21, 0x02, 0, 0, 0, 0});
}
bool KeyaCanDriver::requestMotorSpeed()
{
  return send({0x40, 0x03, 0x21, 0x01, 0, 0, 0, 0});
}
bool KeyaCanDriver::requestMotorCurrent()
{
  return send({0x40, 0x00, 0x21, 0x01, 0, 0, 0, 0});
}
bool KeyaCanDriver::requestFault()
{
  return send({0x40, 0x12, 0x21, 0x01, 0, 0, 0, 0});
}
bool KeyaCanDriver::requestBusVoltage()
{
  return send({0x40, 0x0D, 0x21, 0x02, 0, 0, 0, 0});
}
bool KeyaCanDriver::requestDriverTemperature()
{
  return send({0x40, 0x0F, 0x21, 0x01, 0, 0, 0, 0});
}

void KeyaCanDriver::receiveAvailableFrames()
{
  if (socket_fd_ < 0) return;
  can_frame frame{};
  while (read(socket_fd_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame))) {
    parseFrame(frame);
  }
}
void KeyaCanDriver::parseFrameForTest(
  uint32_t can_id, const std::array<uint8_t, 8> & data)
{
  can_frame frame{};
  frame.can_id = can_id | CAN_EFF_FLAG;
  frame.can_dlc = 8;
  std::memcpy(frame.data, data.data(), 8);
  parseFrame(frame);
}
void KeyaCanDriver::parseFrame(const can_frame & frame)
{
  const uint32_t id = frame.can_id & CAN_EFF_MASK;
  if (frame.can_dlc != 8 || (id != feedback_id_ && id != heartbeat_id_)) return;
  state_.last_feedback = std::chrono::steady_clock::now();
  state_.communication_ok = true;
  if (id == heartbeat_id_) {
    // The manual defines this as 360-count/multi-turn data. It is communication evidence,
    // but the 10,000-count query below remains the authoritative steering position source.
    state_.motor_speed_rpm = be16(&frame.data[2]);
    state_.motor_current_a = be16(&frame.data[4]);
    state_.fault_bits = static_cast<uint16_t>(
      (static_cast<uint16_t>(frame.data[6]) << 8) | frame.data[7]);
    state_.speed_valid = state_.current_valid = true;
    return;
  }
  if (frame.data[0] == 0x63 && frame.data[1] == 0x0D && frame.data[2] == 0x20 &&
    frame.data[3] == 0x31)
  {
    state_.absolute_position_mode_acknowledged = true;
    return;
  }
  if (frame.data[0] != 0x60) return;
  const uint8_t index_lo = frame.data[1], index_hi = frame.data[2], sub = frame.data[3];
  if (index_lo == 0x04 && index_hi == 0x21 && sub == 0x02) {
    state_.motor_position_count = be32(&frame.data[4]);
    state_.position_valid = true;
    state_.last_position_feedback = state_.last_feedback;
  } else if (index_lo == 0x03 && index_hi == 0x21 && sub == 0x01) {
    state_.motor_speed_rpm = be32(&frame.data[4]);
    state_.speed_valid = true;
  } else if (index_lo == 0x00 && index_hi == 0x21 && sub == 0x01) {
    state_.motor_current_a = frame.data[7];
    state_.current_valid = true;
  } else if (index_lo == 0x12 && index_hi == 0x21 && sub == 0x01) {
    state_.fault_bits = static_cast<uint16_t>((frame.data[4] << 8) | frame.data[5]);
  } else if (index_lo == 0x0D && index_hi == 0x21 && sub == 0x02) {
    state_.bus_voltage_v = frame.data[7];
    state_.voltage_valid = true;
  } else if (index_lo == 0x0F && index_hi == 0x21 && sub == 0x01) {
    state_.driver_temperature_c = frame.data[6];
    state_.temperature_valid = true;
  }
  // ACK frames confirm receipt only. The datasheet does not define a trustworthy actual
  // enable-state query, so enabled_feedback_valid deliberately remains false.
}
}  // namespace lateral_ctrl
