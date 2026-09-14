#ifndef DASHBOARD_HOST_RUNTIME_H
#define DASHBOARD_HOST_RUNTIME_H

#include <stdint.h>
#include <exception>

#include "Arduino_CAN.h"

namespace DashboardHostRuntime
{

class StopException : public std::exception {};

struct Event
{
  uint32_t time_ms;
  uint16_t region;
  uint16_t boundary;
  uint8_t kind;
  uint8_t page;
  uint8_t alive;
  uint8_t flags;
  uint8_t direction_auto;
};

struct StartupFrameEvent
{
  uint32_t time_us;
  uint16_t elapsed_ms;
  uint8_t done;
};

void reset();
void requestStop();
bool stopRequested();
void setAllowedTimeUs(uint64_t time_us);
bool waitUntilUs(uint64_t time_us);
uint64_t nowUs();
void consumeNanoseconds(uint64_t nanoseconds);
void idleLoop();
void setReady();
bool ready();
void setError(const char *message);
const char *lastError();
void setTimingProfile(const uint32_t state_us[3],
                      const uint32_t row_us[3][320],
                      uint32_t spi_call_us,
                      uint32_t loop_us);
void setTransportProfile(uint32_t transaction_begin_ns,
                         uint32_t transaction_end_ns,
                         uint32_t spi_byte_extra_ns,
                         const uint32_t spi_buffer_extra_ns[257],
                         const uint32_t rgb565_pack_ns[129],
                         const uint32_t solid_pack_ns[129]);
void setStartupProfile(const uint16_t *elapsed_ms,
                       const uint32_t *state_us,
                       const uint32_t *row_us,
                       uint16_t count);
void chargeState(uint8_t page);
void chargeRow(uint8_t page, uint16_t y);
void chargeTransportTransactionBegin();
void chargeTransportTransactionEnd();
void chargeTransportRgb565Pack(uint16_t pixel_count);
void chargeTransportSolidPack(uint16_t pixel_count);
void chargeTransportSpiByte();
void chargeTransportSpiBuffer(uint16_t bytes);
void recordStartup(bool done, uint32_t time_us);
uint32_t startupStartUs();
uint32_t startupCompletedUs();
void recordStartupFrame(bool done, uint16_t elapsed_ms, uint32_t time_us);
size_t copyStartupFrames(StartupFrameEvent *destination, size_t capacity);
void spiTransfer(const uint8_t *data, size_t length, uint32_t clock_hz);
void setPin(uint8_t pin, uint8_t value);
void copyFramebuffer(uint16_t *destination);
void scheduleCan(uint64_t arrival_us, uint32_t identifier, const uint8_t payload[8]);
int canAvailable();
CanMsg canRead();
void recordRegion(uint8_t page, uint16_t region, uint32_t time_ms);
void recordMode(bool done,
                bool direction_auto,
                uint16_t boundary,
                uint8_t alive,
                uint8_t flags,
                uint32_t time_ms);
void clearEvents(uint16_t region_mask);
size_t copyEvents(Event *destination, size_t capacity);

}  // namespace DashboardHostRuntime

#endif  // DASHBOARD_HOST_RUNTIME_H
