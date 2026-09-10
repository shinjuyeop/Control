#include "include/dashboard_serial.h"

#include <string.h>

namespace
{
#if defined(DASHBOARD_MODE_CHANGE_LOG) || defined(DASHBOARD_TIMING_PROFILE) || \
    defined(DASHBOARD_TIMING_EVENTS_ONLY) || \
    defined(DASHBOARD_TIMING_TRANSPORT_PROFILE) || \
    defined(DASHBOARD_TIMING_STARTUP_PROFILE)
static constexpr uint8_t QUEUE_SIZE = 8U;
#else
static constexpr uint8_t QUEUE_SIZE = 4U;
#endif
char messages[QUEUE_SIZE][DashboardSerial::MESSAGE_SIZE] = {};
uint8_t lengths[QUEUE_SIZE] = {};
uint8_t head = 0U;
uint8_t count = 0U;
uint8_t offset = 0U;
}

namespace DashboardSerial
{

bool enqueue(const char *message)
{
  if (message == nullptr || count >= QUEUE_SIZE) return false;
  const uint8_t slot = (uint8_t)((head + count) % QUEUE_SIZE);
  size_t length = strlen(message);
  if (length > MESSAGE_SIZE - 2U) length = MESSAGE_SIZE - 2U;
  memcpy(messages[slot], message, length);
  messages[slot][length++] = '\n';
  messages[slot][length] = '\0';
  lengths[slot] = (uint8_t)length;
  count++;
  return true;
}

bool hasCapacity()
{
  return count < QUEUE_SIZE;
}

bool idle()
{
  return count == 0U;
}

void service()
{
  if (count == 0U) return;
  const int available = Serial.availableForWrite();
  if (available <= 0) return;
  const uint8_t remaining = (uint8_t)(lengths[head] - offset);
  const size_t available_bytes = (size_t)available;
  const size_t chunk = remaining < available_bytes
                           ? remaining : available_bytes;
  if (chunk == 0U) return;
  const size_t written = Serial.write(
      (const uint8_t *)&messages[head][offset], chunk);
  offset = (uint8_t)(offset + written);
  if (offset >= lengths[head])
  {
    offset = 0U;
    head = (uint8_t)((head + 1U) % QUEUE_SIZE);
    count--;
  }
}

}  // namespace DashboardSerial
