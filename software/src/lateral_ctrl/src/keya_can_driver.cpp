#include "lateral_ctrl/keya_can_driver.hpp"

#include "keya_can_frame_decoder.hpp"

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
#include <stdexcept>

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

namespace detail
{
bool applyKeyaCanFrame(
  KeyaMotorState & state, uint32_t feedback_id, uint32_t heartbeat_id,
  uint32_t can_id, uint8_t dlc, const uint8_t data[8],
  std::chrono::steady_clock::time_point received_at)
{
  if (dlc != 8 || (can_id != feedback_id && can_id != heartbeat_id)) return false;
  state.last_feedback = received_at;
  state.communication_ok = true;
  if (can_id == heartbeat_id) {
    // The manual defines this as 360-count/multi-turn data. It is communication evidence,
    // but the 10,000-count query below remains the authoritative steering position source.
    state.motor_speed_rpm = be16(&data[2]);
    state.motor_current_a = be16(&data[4]);
    state.fault_bits = static_cast<uint16_t>(
      (static_cast<uint16_t>(data[6]) << 8) | data[7]);
    state.fault_valid = true;
    state.last_fault_feedback = received_at;
    state.speed_valid = state.current_valid = true;
    return true;
  }
  if (data[0] != 0x60) return true;
  const uint8_t index_lo = data[1], index_hi = data[2], sub = data[3];
  if (index_lo == 0x04 && index_hi == 0x21 && sub == 0x02) {
    state.motor_position_count = be32(&data[4]);
    state.position_valid = true;
    state.last_position_feedback = state.last_feedback;
  } else if (index_lo == 0x03 && index_hi == 0x21 && sub == 0x01) {
    state.motor_speed_rpm = be32(&data[4]);
    state.speed_valid = true;
  } else if (index_lo == 0x00 && index_hi == 0x21 && sub == 0x01) {
    state.motor_current_a = data[7];
    state.current_valid = true;
  } else if (index_lo == 0x12 && index_hi == 0x21 && sub == 0x01) {
    state.fault_bits = static_cast<uint16_t>((data[4] << 8) | data[5]);
    state.fault_valid = true;
    state.last_fault_feedback = received_at;
  } else if (index_lo == 0x0D && index_hi == 0x21 && sub == 0x02) {
    state.bus_voltage_v = data[7];
    state.voltage_valid = true;
  } else if (index_lo == 0x0F && index_hi == 0x21 && sub == 0x01) {
    state.driver_temperature_c = data[6];
    state.temperature_valid = true;
  }
  // ACK frames confirm receipt only. The datasheet does not define a trustworthy actual
  // enable-state query, so enabled_feedback_valid deliberately remains false.
  return true;
}
}  // namespace detail

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
  const int timestamping = 1;
  if (setsockopt(socket_fd_, SOL_SOCKET, SO_TIMESTAMPNS, &timestamping, sizeof(timestamping)) < 0) {
    close();
    return false;
  }
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
  const int interface_index = ifr.ifr_ifindex;
  if (ioctl(socket_fd_, SIOCGIFFLAGS, &ifr) < 0 || !(ifr.ifr_flags & IFF_UP)) {
    close(); return false;
  }
  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = interface_index;
  if (bind(socket_fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) < 0) {
    close(); return false;
  }
  return true;
}
void KeyaCanDriver::close()
{
  if (socket_fd_ >= 0) ::close(socket_fd_);
  socket_fd_ = -1;
  // Preserve raw faults as evidence, but never treat cached samples as fresh.
  state_.communication_ok = state_.position_valid = state_.fault_valid = false;
  state_.last_feedback = state_.last_position_feedback = state_.last_fault_feedback = {};
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
std::array<uint8_t, 8> KeyaCanDriver::torquePayload(int32_t command)
{
  if (command < -10000 || command > 10000) {
    throw std::out_of_range("KEYA torque command must be in [-10000, 10000]");
  }
  const auto value = static_cast<uint32_t>(command);
  return {0x23, 0x01, 0x20, 0x01, static_cast<uint8_t>(value >> 24),
    static_cast<uint8_t>(value >> 16), static_cast<uint8_t>(value >> 8),
    static_cast<uint8_t>(value)};
}
bool KeyaCanDriver::send(const std::array<uint8_t, 8> & data)
{
  if (socket_fd_ < 0) return false;
  can_frame frame{};
  frame.can_id = command_id_ | CAN_EFF_FLAG;
  frame.can_dlc = 8;
  std::memcpy(frame.data, data.data(), data.size());
  const bool ok = write(socket_fd_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
  if (!ok) close();
  return ok;
}
bool KeyaCanDriver::sendEnable() { return send(enablePayload()); }
bool KeyaCanDriver::sendDisable() { return send(disablePayload()); }
bool KeyaCanDriver::sendPositionCount(int32_t count) { return send(positionPayload(count)); }
bool KeyaCanDriver::sendTorqueCommand(int32_t command)
{
  if (command < -10000 || command > 10000) return false;
  return send(torquePayload(command));
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
  while (true) {
    can_frame frame{};
    iovec iov{&frame, sizeof(frame)};
    alignas(cmsghdr) char control[CMSG_SPACE(sizeof(timespec))]{};
    msghdr message{};
    message.msg_iov = &iov;
    message.msg_iovlen = 1;
    message.msg_control = control;
    message.msg_controllen = sizeof(control);
    const auto size = recvmsg(socket_fd_, &message, MSG_DONTWAIT);
    if (size != static_cast<ssize_t>(sizeof(frame))) {
      if (size < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) close();
      break;
    }
    if (!(frame.can_id & CAN_EFF_FLAG) || (frame.can_id & (CAN_RTR_FLAG | CAN_ERR_FLAG))) continue;
    // Timestamp arrival, not queue-drain time: queued feedback must not look fresh,
    // and the wheel derivative must follow real sample intervals.
    auto received_at = std::chrono::steady_clock::time_point{};
    for (auto * cmsg = CMSG_FIRSTHDR(&message); cmsg; cmsg = CMSG_NXTHDR(&message, cmsg)) {
      if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMPNS &&
        cmsg->cmsg_len >= CMSG_LEN(sizeof(timespec)))
      {
        timespec stamp{};
        std::memcpy(&stamp, CMSG_DATA(cmsg), sizeof(stamp));
        const auto arrival = std::chrono::system_clock::time_point(
          std::chrono::seconds(stamp.tv_sec) + std::chrono::nanoseconds(stamp.tv_nsec));
        const auto age = std::chrono::system_clock::now() - arrival;
        if (age < std::chrono::system_clock::duration::zero()) break;
        received_at = std::chrono::steady_clock::now() -
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(age);
      }
    }
    if (received_at.time_since_epoch().count() == 0) continue;
    detail::applyKeyaCanFrame(
      state_, feedback_id_, heartbeat_id_, frame.can_id & CAN_EFF_MASK,
      frame.can_dlc, frame.data, received_at);
  }
}
}  // namespace lateral_ctrl
