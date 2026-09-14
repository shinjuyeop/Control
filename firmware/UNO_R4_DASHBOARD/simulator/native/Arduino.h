#ifndef DASHBOARD_HOST_ARDUINO_H
#define DASHBOARD_HOST_ARDUINO_H

#include <stddef.h>
#include <stdint.h>

static constexpr uint8_t LOW = 0U;
static constexpr uint8_t HIGH = 1U;
static constexpr uint8_t OUTPUT = 1U;
static constexpr int HEX = 16;
static constexpr uint8_t D7 = 7U;
static constexpr uint8_t D8 = 8U;
static constexpr uint8_t D9 = 9U;
static constexpr uint8_t D10 = 10U;

struct __FlashStringHelper;
#define F(value) reinterpret_cast<const __FlashStringHelper *>(value)

uint32_t millis();
uint32_t micros();
void delay(uint32_t milliseconds);
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);

class HostSerial
{
 public:
  void begin(unsigned long) {}
  void print(const __FlashStringHelper *) {}
  void print(const char *) {}
  void print(char) {}
  void print(unsigned char, int = 10) {}
  void print(int, int = 10) {}
  void print(unsigned int, int = 10) {}
  void print(long, int = 10) {}
  void print(unsigned long, int = 10) {}
  void println() {}
  template <typename T> void println(const T &value) { print(value); }
  int availableForWrite() { return 64; }
  size_t write(const uint8_t *, size_t length) { return length; }
};

extern HostSerial Serial;

#endif  // DASHBOARD_HOST_ARDUINO_H
