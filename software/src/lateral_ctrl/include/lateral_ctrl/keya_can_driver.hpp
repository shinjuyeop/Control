#pragma once

#include "lateral_ctrl/steering_types.hpp"

#include <array>
#include <cstdint>
#include <string>

struct can_frame;

namespace lateral_ctrl
{
class KeyaCanDriver
{
public:
  KeyaCanDriver(std::string interface_name, uint8_t node_id);
  ~KeyaCanDriver();
  bool open();
  void close();
  bool isOpen() const { return socket_fd_ >= 0; }
  bool sendEnable();
  bool sendDisable();
  bool sendPositionCount(int32_t count);
  bool sendAbsolutePositionMode();
  bool requestEncoderPosition();
  bool requestMotorSpeed();
  bool requestMotorCurrent();
  bool requestFault();
  bool requestBusVoltage();
  bool requestDriverTemperature();
  void receiveAvailableFrames();
  const KeyaMotorState & motorState() const { return state_; }

  static std::array<uint8_t, 8> enablePayload();
  static std::array<uint8_t, 8> disablePayload();
  static std::array<uint8_t, 8> positionPayload(int32_t count);
  static std::array<uint8_t, 8> absolutePositionModePayload();
  void parseFrameForTest(uint32_t can_id, const std::array<uint8_t, 8> & data);

private:
  bool send(const std::array<uint8_t, 8> & data);
  void parseFrame(const can_frame & frame);
  std::string interface_name_;
  int socket_fd_{-1};
  uint32_t command_id_, feedback_id_, heartbeat_id_;
  KeyaMotorState state_;
};
}  // namespace lateral_ctrl
