#ifndef DASHBOARD_HOST_ARDUINO_CAN_H
#define DASHBOARD_HOST_ARDUINO_CAN_H

#include "Arduino.h"

enum class CanBitRate : uint8_t
{
  BR_250k
};

class CanMsg
{
 public:
  CanMsg() = default;
  CanMsg(uint32_t identifier, const uint8_t payload[8]);
  CanMsg(uint32_t identifier, uint8_t length, const uint8_t *payload);
  bool isStandardId() const { return true; }
  uint32_t getStandardId() const { return identifier_; }

  uint8_t data[8] = {};
  uint8_t data_length = 8U;

 private:
  uint32_t identifier_ = 0U;
};

class HostCAN
{
 public:
  bool begin(CanBitRate) { ready_ = true; return true; }
  void end() { ready_ = false; }
  int available();
  CanMsg read();

 private:
  bool ready_ = false;
};

extern HostCAN CAN;

#endif  // DASHBOARD_HOST_ARDUINO_CAN_H
