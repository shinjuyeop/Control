#pragma once
#include "Arduino.h"
constexpr uint8_t MSBFIRST=1, SPI_MODE3=3;
struct SPISettings { SPISettings(uint32_t, uint8_t, uint8_t) {} };
struct DisplayPixelSink {
  void begin() {}
  void beginTransaction(const SPISettings&) {}
  void endTransaction() {}
  uint8_t transfer(uint8_t value);
  void transfer(void *buffer, size_t length);
};
extern DisplayPixelSink SPI;
