#include "ft6336_touch.h"

#include <Arduino.h>
#include <Wire.h>

namespace
{
constexpr uint8_t ADDRESS = 0x38;
constexpr uint8_t RESET_PIN = D3;
constexpr uint8_t INT_PIN = D2;
constexpr uint32_t POLL_MS = 20;
constexpr uint32_t RETRY_MS = 1000;
Ft6336Touch::State current{};
volatile uint32_t irq_edges = 0;
uint32_t last_poll_ms = 0;

void onInterrupt()
{
  ++irq_edges;
}

bool readRegisters(uint8_t reg, uint8_t *data, size_t size)
{
  Wire.beginTransmission(ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(ADDRESS, size) != size) return false;
  for (size_t i = 0; i < size; ++i) data[i] = uint8_t(Wire.read());
  return true;
}

bool readIdentity()
{
  uint8_t cipher[2];
  if (!readRegisters(0x9F, cipher, sizeof(cipher)) ||
      !readRegisters(0xA3, &current.chip_high, 1) ||
      !readRegisters(0xA6, &current.firmware, 1) ||
      !readRegisters(0xA8, &current.vendor, 1)) return false;
  current.chip_mid = cipher[0];
  current.chip_low = cipher[1];
  return true;
}
}

namespace Ft6336Touch
{
void begin()
{
  // MSP4031 translates SDA/SCL and RST. INT is directly connected to the
  // 3.3 V touch controller: do not enable the UNO's 5 V input pull-up.
  pinMode(INT_PIN, INPUT);
  digitalWrite(RESET_PIN, LOW);
  pinMode(RESET_PIN, OUTPUT);
  delay(20);
  digitalWrite(RESET_PIN, HIGH);
  delay(500);  // LCDWIKI reset sequence; datasheet requires >=300 ms startup.
  Wire.begin();  // UNO R4 Minima A4=SDA, A5=SCL.
  Wire.setClock(100000);
  Wire.setWireTimeout(3000);
  attachInterrupt(digitalPinToInterrupt(INT_PIN), onInterrupt, FALLING);
  current.identified = readIdentity();
  if (!current.identified) ++current.errors;
  last_poll_ms = millis() - POLL_MS;
}

void poll(uint32_t now_ms)
{
  current.irq_level = uint8_t(digitalRead(INT_PIN));
  current.irq_edges = irq_edges;
  const uint32_t period = current.valid || current.samples == 0 ? POLL_MS : RETRY_MS;
  if (uint32_t(now_ms - last_poll_ms) < period) return;
  last_poll_ms = now_ms;
  if (!current.identified) current.identified = readIdentity();

  // One contiguous I2C read: TD_STATUS plus both six-byte point records.
  // Polling also detects release if INT is absent or a pulse was missed.
  uint8_t data[13];
  const bool received = current.identified && readRegisters(0x02, data, sizeof(data));
  current.valid = received && (data[0] & 0x0F) <= 2;
  ++current.samples;
  current.count = 0;
  if (!current.valid)
  {
    ++current.errors;
    return;
  }
  current.sample_ms = now_ms;
  current.count = data[0] & 0x0F;
  for (uint8_t i = 0; i < current.count; ++i)
  {
    const uint8_t *point = &data[1 + 6 * i];
    current.points[i] = {
      uint16_t((uint16_t(point[0] & 0x0F) << 8) | point[1]),
      uint16_t((uint16_t(point[2] & 0x0F) << 8) | point[3]),
      uint8_t(point[2] >> 4), uint8_t(point[0] >> 6)
    };
  }
}

const State &state()
{
  return current;
}
}
