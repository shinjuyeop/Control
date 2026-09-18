#pragma once

#include "lateral_ctrl/steering_types.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace lateral_ctrl
{
class KeyaCanDriver
{
public:
  KeyaCanDriver(std::string interface_name, uint8_t node_id);
  virtual ~KeyaCanDriver();
  virtual bool open();
  void close();
  virtual bool isOpen() const { return socket_fd_ >= 0; }
  virtual bool sendEnable();
  virtual bool sendDisable();
  virtual bool sendPositionCount(int32_t count);
  virtual bool sendTorqueCommand(int32_t command);
  virtual bool requestEncoderPosition();
  bool requestMotorSpeed();
  bool requestMotorCurrent();
  bool requestFault();
  bool requestBusVoltage();
  bool requestDriverTemperature();
  virtual void receiveAvailableFrames();
  virtual const KeyaMotorState & motorState() const { return state_; }

  static std::array<uint8_t, 8> enablePayload();
  static std::array<uint8_t, 8> disablePayload();
  static std::array<uint8_t, 8> positionPayload(int32_t count);
  static std::array<uint8_t, 8> torquePayload(int32_t command);

private:
  friend class KeyaCanDriverTestPeer;
  bool send(const std::array<uint8_t, 8> & data);
  std::string interface_name_;
  int socket_fd_{-1};
  uint32_t command_id_, feedback_id_, heartbeat_id_;
  KeyaMotorState state_;
};
}  // namespace lateral_ctrl
