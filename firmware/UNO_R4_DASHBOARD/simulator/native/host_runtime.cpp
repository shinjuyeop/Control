#include "host_runtime.h"

#include <algorithm>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <string.h>

#include "SPI.h"

namespace
{

static constexpr uint16_t LCD_WIDTH = 480U;
static constexpr uint16_t LCD_HEIGHT = 320U;
static constexpr uint8_t LCD_CS_PIN = D10;
static constexpr uint8_t LCD_DC_PIN = D9;
static constexpr uint8_t LCD_BL_PIN = D7;

struct ScheduledCan
{
  uint64_t arrival_ns;
  uint32_t identifier;
  uint8_t payload[8];
};

std::mutex runtime_mutex;
std::condition_variable runtime_cv;
uint64_t now_ns = 0U;
uint64_t allowed_ns = 0U;
bool stop_requested = false;
bool firmware_ready = false;
bool worker_paused = false;
bool worker_failed = false;
std::string worker_error;
uint8_t pin_values[32] = {};
uint16_t framebuffer[LCD_WIDTH * LCD_HEIGHT] = {};
std::deque<ScheduledCan> can_queue;
static constexpr size_t EVENT_CAPACITY = 256U;
DashboardHostRuntime::Event events[EVENT_CAPACITY] = {};
size_t event_count = 0U;
uint16_t event_region_mask = 0xFFFFU;
uint32_t startup_start_us = 0U;
uint32_t startup_completed_us = 0U;

uint8_t current_command = 0U;
uint8_t command_data[4] = {};
uint8_t command_data_count = 0U;
uint16_t window_x0 = 0U;
uint16_t window_y0 = 0U;
uint16_t window_x1 = LCD_WIDTH - 1U;
uint16_t window_y1 = LCD_HEIGHT - 1U;
uint16_t cursor_x = 0U;
uint16_t cursor_y = 0U;
uint8_t pixel_high = 0U;
bool have_pixel_high = false;

uint32_t profile_state_us[3] = {};
uint32_t profile_row_us[3][LCD_HEIGHT] = {};
uint32_t profile_spi_call_us = 0U;
uint32_t profile_loop_us = 10U;
uint32_t profile_transaction_begin_ns = 0U;
uint32_t profile_transaction_end_ns = 0U;
uint32_t profile_spi_byte_extra_ns = 0U;
uint32_t profile_spi_buffer_extra_ns[257] = {};
uint32_t profile_rgb565_pack_ns[129] = {};
uint32_t profile_solid_pack_ns[129] = {};
static constexpr uint16_t STARTUP_PROFILE_CAPACITY = 141U;
uint16_t profile_startup_elapsed_ms[STARTUP_PROFILE_CAPACITY] = {};
uint32_t profile_startup_state_us[STARTUP_PROFILE_CAPACITY] = {};
uint32_t profile_startup_row_us[STARTUP_PROFILE_CAPACITY][LCD_HEIGHT] = {};
uint16_t profile_startup_count = 0U;
uint16_t current_startup_elapsed_ms = 0U;
static constexpr size_t STARTUP_FRAME_EVENT_CAPACITY = 64U;
DashboardHostRuntime::StartupFrameEvent
    startup_frame_events[STARTUP_FRAME_EVENT_CAPACITY] = {};
size_t startup_frame_event_count = 0U;

uint32_t interpolatedStartupUs(const uint32_t *values, size_t stride)
{
  if (profile_startup_count == 0U) return 0U;
  const uint16_t elapsed_ms = current_startup_elapsed_ms;
  if (elapsed_ms <= profile_startup_elapsed_ms[0]) return values[0];
  const uint16_t last = (uint16_t)(profile_startup_count - 1U);
  if (elapsed_ms >= profile_startup_elapsed_ms[last])
    return values[(size_t)last * stride];
  uint16_t upper = 1U;
  while (profile_startup_elapsed_ms[upper] < elapsed_ms) upper++;
  const uint16_t lower = (uint16_t)(upper - 1U);
  const uint16_t x0 = profile_startup_elapsed_ms[lower];
  const uint16_t x1 = profile_startup_elapsed_ms[upper];
  const uint32_t y0 = values[(size_t)lower * stride];
  const uint32_t y1 = values[(size_t)upper * stride];
  const uint32_t left = (uint32_t)(x1 - elapsed_ms);
  const uint32_t right = (uint32_t)(elapsed_ms - x0);
  return (uint32_t)(((uint64_t)y0 * left + (uint64_t)y1 * right +
                     (x1 - x0) / 2U) /
                    (x1 - x0));
}

void parseSpiByte(uint8_t value)
{
  if (pin_values[LCD_CS_PIN] != LOW) return;
  if (pin_values[LCD_DC_PIN] == LOW)
  {
    current_command = value;
    command_data_count = 0U;
    have_pixel_high = false;
    if (value == 0x2CU)
    {
      cursor_x = window_x0;
      cursor_y = window_y0;
    }
    return;
  }
  if (current_command == 0x2AU || current_command == 0x2BU)
  {
    if (command_data_count < 4U) command_data[command_data_count++] = value;
    if (command_data_count == 4U)
    {
      const uint16_t first =
          (uint16_t)((uint16_t)command_data[0] << 8) | command_data[1];
      const uint16_t last =
          (uint16_t)((uint16_t)command_data[2] << 8) | command_data[3];
      if (current_command == 0x2AU)
      {
        window_x0 = std::min<uint16_t>(first, LCD_WIDTH - 1U);
        window_x1 = std::min<uint16_t>(last, LCD_WIDTH - 1U);
      }
      else
      {
        window_y0 = std::min<uint16_t>(first, LCD_HEIGHT - 1U);
        window_y1 = std::min<uint16_t>(last, LCD_HEIGHT - 1U);
      }
    }
    return;
  }
  if (current_command != 0x2CU) return;
  if (!have_pixel_high)
  {
    pixel_high = value;
    have_pixel_high = true;
    return;
  }
  have_pixel_high = false;
  if (cursor_x < LCD_WIDTH && cursor_y < LCD_HEIGHT)
    framebuffer[(size_t)cursor_y * LCD_WIDTH + cursor_x] =
        (uint16_t)((uint16_t)pixel_high << 8) | value;
  if (cursor_x < window_x1) cursor_x++;
  else
  {
    cursor_x = window_x0;
    if (cursor_y < window_y1) cursor_y++;
    else cursor_y = window_y0;
  }
}

}  // namespace

HostSerial Serial;
HostSPI SPI;
HostCAN CAN;

uint32_t millis() { return (uint32_t)(DashboardHostRuntime::nowUs() / 1000U); }
uint32_t micros() { return (uint32_t)DashboardHostRuntime::nowUs(); }
void delay(uint32_t milliseconds)
{
  DashboardHostRuntime::consumeNanoseconds((uint64_t)milliseconds * 1000000U);
}
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t pin, uint8_t value)
{
  DashboardHostRuntime::setPin(pin, value);
}

void HostSPI::beginTransaction(const SPISettings &settings)
{
  clock_hz_ = settings.clock_hz;
}
uint8_t HostSPI::transfer(uint8_t value)
{
  DashboardHostRuntime::spiTransfer(&value, 1U, clock_hz_);
  return 0U;
}
void HostSPI::transfer(void *data, size_t length)
{
  DashboardHostRuntime::spiTransfer((const uint8_t *)data, length, clock_hz_);
  memset(data, 0, length);
}

CanMsg::CanMsg(uint32_t identifier, const uint8_t payload[8])
    : identifier_(identifier)
{
  memcpy(data, payload, sizeof(data));
}
CanMsg::CanMsg(uint32_t identifier, uint8_t length, const uint8_t *payload)
    : identifier_(identifier)
{
  data_length = length > 8U ? 8U : length;
  if (payload != nullptr) memcpy(data, payload, data_length);
}
int HostCAN::available()
{
  return ready_ ? DashboardHostRuntime::canAvailable() : 0;
}
CanMsg HostCAN::read()
{
  return DashboardHostRuntime::canRead();
}

namespace DashboardHostRuntime
{

void reset()
{
  std::lock_guard<std::mutex> lock(runtime_mutex);
  now_ns = 0U;
  allowed_ns = 0U;
  stop_requested = false;
  firmware_ready = false;
  worker_paused = false;
  worker_failed = false;
  worker_error.clear();
  memset(pin_values, 0, sizeof(pin_values));
  memset(framebuffer, 0, sizeof(framebuffer));
  can_queue.clear();
  event_count = 0U;
  startup_start_us = 0U;
  startup_completed_us = 0U;
  startup_frame_event_count = 0U;
  current_startup_elapsed_ms = 0U;
  current_command = 0U;
  command_data_count = 0U;
  window_x0 = 0U;
  window_y0 = 0U;
  window_x1 = LCD_WIDTH - 1U;
  window_y1 = LCD_HEIGHT - 1U;
  cursor_x = 0U;
  cursor_y = 0U;
  have_pixel_high = false;
}

void requestStop()
{
  std::lock_guard<std::mutex> lock(runtime_mutex);
  stop_requested = true;
  runtime_cv.notify_all();
}

bool stopRequested()
{
  std::lock_guard<std::mutex> lock(runtime_mutex);
  return stop_requested;
}

void setAllowedTimeUs(uint64_t time_us)
{
  std::lock_guard<std::mutex> lock(runtime_mutex);
  if (time_us * 1000U > allowed_ns) allowed_ns = time_us * 1000U;
  runtime_cv.notify_all();
}

bool waitUntilUs(uint64_t time_us)
{
  std::unique_lock<std::mutex> lock(runtime_mutex);
  const uint64_t target_ns = time_us * 1000U;
  runtime_cv.wait(lock, [&] {
    return (now_ns >= target_ns && worker_paused) ||
           worker_failed || stop_requested;
  });
  return !worker_failed && !stop_requested;
}

uint64_t nowUs()
{
  std::lock_guard<std::mutex> lock(runtime_mutex);
  return now_ns / 1000U;
}

void consumeNanoseconds(uint64_t nanoseconds)
{
  std::unique_lock<std::mutex> lock(runtime_mutex);
  while (nanoseconds != 0U)
  {
    if (stop_requested) throw StopException();
    if (now_ns >= allowed_ns)
    {
      worker_paused = true;
      runtime_cv.notify_all();
      runtime_cv.wait(lock, [] { return stop_requested || allowed_ns > now_ns; });
      worker_paused = false;
      continue;
    }
    const uint64_t available = allowed_ns - now_ns;
    const uint64_t step = std::min(nanoseconds, available);
    now_ns += step;
    nanoseconds -= step;
    runtime_cv.notify_all();
  }
}

void idleLoop()
{
  consumeNanoseconds((uint64_t)profile_loop_us * 1000U);
}

void setReady()
{
  std::lock_guard<std::mutex> lock(runtime_mutex);
  firmware_ready = true;
  runtime_cv.notify_all();
}

bool ready()
{
  std::lock_guard<std::mutex> lock(runtime_mutex);
  return firmware_ready;
}

void setError(const char *message)
{
  std::lock_guard<std::mutex> lock(runtime_mutex);
  worker_failed = true;
  worker_error = message == nullptr ? "unknown native runtime error" : message;
  runtime_cv.notify_all();
}

const char *lastError()
{
  std::lock_guard<std::mutex> lock(runtime_mutex);
  return worker_error.c_str();
}

void setTimingProfile(const uint32_t state_us[3],
                      const uint32_t row_us[3][320],
                      uint32_t spi_call_us,
                      uint32_t loop_us)
{
  memcpy(profile_state_us, state_us, sizeof(profile_state_us));
  memcpy(profile_row_us, row_us, sizeof(profile_row_us));
  profile_spi_call_us = spi_call_us;
  profile_loop_us = loop_us == 0U ? 10U : loop_us;
}

void setTransportProfile(uint32_t transaction_begin_ns,
                         uint32_t transaction_end_ns,
                         uint32_t spi_byte_extra_ns,
                         const uint32_t spi_buffer_extra_ns[257],
                         const uint32_t rgb565_pack_ns[129],
                         const uint32_t solid_pack_ns[129])
{
  profile_transaction_begin_ns = transaction_begin_ns;
  profile_transaction_end_ns = transaction_end_ns;
  profile_spi_byte_extra_ns = spi_byte_extra_ns;
  memcpy(profile_spi_buffer_extra_ns, spi_buffer_extra_ns,
         sizeof(profile_spi_buffer_extra_ns));
  memcpy(profile_rgb565_pack_ns, rgb565_pack_ns,
         sizeof(profile_rgb565_pack_ns));
  memcpy(profile_solid_pack_ns, solid_pack_ns,
         sizeof(profile_solid_pack_ns));
}

void setStartupProfile(const uint16_t *elapsed_ms,
                       const uint32_t *state_us,
                       const uint32_t *row_us,
                       uint16_t count)
{
  profile_startup_count = std::min<uint16_t>(count, STARTUP_PROFILE_CAPACITY);
  if (profile_startup_count == 0U) return;
  memcpy(profile_startup_elapsed_ms, elapsed_ms,
         profile_startup_count * sizeof(profile_startup_elapsed_ms[0]));
  memcpy(profile_startup_state_us, state_us,
         profile_startup_count * sizeof(profile_startup_state_us[0]));
  memcpy(profile_startup_row_us, row_us,
         (size_t)profile_startup_count * LCD_HEIGHT * sizeof(uint32_t));
}

void chargeState(uint8_t page)
{
  const uint32_t value =
      page == 0U && profile_startup_count != 0U
          ? interpolatedStartupUs(profile_startup_state_us, 1U)
          : page < 3U ? profile_state_us[page] : 0U;
  consumeNanoseconds((uint64_t)value * 1000U);
}

void chargeRow(uint8_t page, uint16_t y)
{
  if (y >= LCD_HEIGHT) return;
  const uint32_t value =
      page == 0U && profile_startup_count != 0U
          ? interpolatedStartupUs(&profile_startup_row_us[0][y], LCD_HEIGHT)
          : page < 3U ? profile_row_us[page][y] : 0U;
  consumeNanoseconds((uint64_t)value * 1000U);
}

void chargeTransportTransactionBegin()
{
  consumeNanoseconds(profile_transaction_begin_ns);
}

void chargeTransportTransactionEnd()
{
  consumeNanoseconds(profile_transaction_end_ns);
}

void chargeTransportRgb565Pack(uint16_t pixel_count)
{
  if (pixel_count <= 128U)
    consumeNanoseconds(profile_rgb565_pack_ns[pixel_count]);
}

void chargeTransportSolidPack(uint16_t pixel_count)
{
  if (pixel_count <= 128U)
    consumeNanoseconds(profile_solid_pack_ns[pixel_count]);
}

void chargeTransportSpiByte()
{
  consumeNanoseconds(profile_spi_byte_extra_ns);
}

void chargeTransportSpiBuffer(uint16_t bytes)
{
  if (bytes <= 256U)
    consumeNanoseconds(profile_spi_buffer_extra_ns[bytes]);
}

void spiTransfer(const uint8_t *data, size_t length, uint32_t clock_hz)
{
  uint64_t consumed_ns = 0U;
  for (size_t index = 0U; index < length; ++index)
  {
    const uint64_t total_ns =
        ((uint64_t)(index + 1U) * 8000000000ULL + clock_hz - 1U) /
        clock_hz;
    consumeNanoseconds(total_ns - consumed_ns);
    consumed_ns = total_ns;
    std::lock_guard<std::mutex> lock(runtime_mutex);
    parseSpiByte(data[index]);
  }
}

void setPin(uint8_t pin, uint8_t value)
{
  std::lock_guard<std::mutex> lock(runtime_mutex);
  if (pin < sizeof(pin_values)) pin_values[pin] = value;
}

void copyFramebuffer(uint16_t *destination)
{
  std::lock_guard<std::mutex> lock(runtime_mutex);
  if (pin_values[LCD_BL_PIN] == LOW)
    memset(destination, 0, sizeof(framebuffer));
  else
    memcpy(destination, framebuffer, sizeof(framebuffer));
}

void scheduleCan(uint64_t arrival_us, uint32_t identifier, const uint8_t payload[8])
{
  std::lock_guard<std::mutex> lock(runtime_mutex);
  ScheduledCan item = {arrival_us * 1000U, identifier, {}};
  memcpy(item.payload, payload, sizeof(item.payload));
  const auto position = std::upper_bound(
      can_queue.begin(), can_queue.end(), item.arrival_ns,
      [](uint64_t value, const ScheduledCan &candidate) {
        return value < candidate.arrival_ns;
      });
  can_queue.insert(position, item);
}

int canAvailable()
{
  std::lock_guard<std::mutex> lock(runtime_mutex);
  int count = 0;
  for (const ScheduledCan &item : can_queue)
  {
    if (item.arrival_ns > now_ns) break;
    count++;
  }
  return count;
}

CanMsg canRead()
{
  std::lock_guard<std::mutex> lock(runtime_mutex);
  if (can_queue.empty() || can_queue.front().arrival_ns > now_ns) return CanMsg();
  const ScheduledCan item = can_queue.front();
  can_queue.pop_front();
  return CanMsg(item.identifier, item.payload);
}

void recordRegion(uint8_t page, uint16_t region, uint32_t time_ms)
{
  std::lock_guard<std::mutex> lock(runtime_mutex);
  if ((region & event_region_mask) != 0U && event_count < EVENT_CAPACITY)
    events[event_count++] = {time_ms, region, 0U, 1U, page, 0U, 0U, 0U};
}

void recordStartup(bool done, uint32_t time_us)
{
  std::lock_guard<std::mutex> lock(runtime_mutex);
  if (done) startup_completed_us = time_us;
  else startup_start_us = time_us;
}

uint32_t startupStartUs()
{
  std::lock_guard<std::mutex> lock(runtime_mutex);
  return startup_start_us;
}

uint32_t startupCompletedUs()
{
  std::lock_guard<std::mutex> lock(runtime_mutex);
  return startup_completed_us;
}

void recordStartupFrame(bool done, uint16_t elapsed_ms, uint32_t time_us)
{
  std::lock_guard<std::mutex> lock(runtime_mutex);
  if (!done) current_startup_elapsed_ms = elapsed_ms;
  if (startup_frame_event_count < STARTUP_FRAME_EVENT_CAPACITY)
  {
    startup_frame_events[startup_frame_event_count++] = {
        time_us, elapsed_ms, (uint8_t)(done ? 1U : 0U)};
  }
}

size_t copyStartupFrames(StartupFrameEvent *destination, size_t capacity)
{
  std::lock_guard<std::mutex> lock(runtime_mutex);
  const size_t count = std::min(startup_frame_event_count, capacity);
  if (count != 0U)
    memcpy(destination, startup_frame_events,
           count * sizeof(StartupFrameEvent));
  return count;
}

void recordMode(bool done,
                bool direction_auto,
                uint16_t boundary,
                uint8_t alive,
                uint8_t flags,
                uint32_t time_ms)
{
  std::lock_guard<std::mutex> lock(runtime_mutex);
  if (event_count < EVENT_CAPACITY)
    events[event_count++] = {
        time_ms, 0U, boundary, (uint8_t)(done ? 3U : 2U),
        0U, alive, flags, (uint8_t)(direction_auto ? 1U : 0U)};
}

void clearEvents(uint16_t region_mask)
{
  std::lock_guard<std::mutex> lock(runtime_mutex);
  event_count = 0U;
  event_region_mask = region_mask;
  startup_start_us = 0U;
  startup_completed_us = 0U;
  startup_frame_event_count = 0U;
}

size_t copyEvents(Event *destination, size_t capacity)
{
  std::lock_guard<std::mutex> lock(runtime_mutex);
  const size_t count = std::min(event_count, capacity);
  if (count != 0U) memcpy(destination, events, count * sizeof(Event));
  return count;
}

}  // namespace DashboardHostRuntime
