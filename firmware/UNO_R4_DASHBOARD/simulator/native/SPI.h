#ifndef DASHBOARD_HOST_SPI_H
#define DASHBOARD_HOST_SPI_H

#include "Arduino.h"

static constexpr uint8_t MSBFIRST = 1U;
static constexpr uint8_t SPI_MODE3 = 3U;

class SPISettings
{
 public:
  SPISettings(uint32_t clock_hz, uint8_t, uint8_t) : clock_hz(clock_hz) {}
  uint32_t clock_hz;
};

class HostSPI
{
 public:
  void begin() {}
  void beginTransaction(const SPISettings &settings);
  void endTransaction() {}
  uint8_t transfer(uint8_t value);
  void transfer(void *data, size_t length);

 private:
  uint32_t clock_hz_ = 12000000U;
};

extern HostSPI SPI;

#endif  // DASHBOARD_HOST_SPI_H
