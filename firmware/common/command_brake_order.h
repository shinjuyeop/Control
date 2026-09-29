#ifndef COMMAND_BRAKE_ORDER_H
#define COMMAND_BRAKE_ORDER_H
#include <stdint.h>

/* Compare Host stamps without comparing MCU and Host clocks. Always honor brake;
 * after a stamped brake, a release must have been produced later. Receipt-age
 * watchdogs remain in their existing owners. No command is replayed here. */
static inline uint8_t CommandBrakeOrder_Accept(
    int64_t *last_brake_ns, uint8_t brake, int32_t sec, uint32_t nanosec)
{
  const int64_t stamp = (sec >= 0 && nanosec < 1000000000u) ?
      (int64_t)sec * 1000000000LL + nanosec : 0;
  if (brake) {
    if (stamp > *last_brake_ns) *last_brake_ns = stamp;
    return 1u;
  }
  return (*last_brake_ns == 0 || stamp > *last_brake_ns) ? 1u : 0u;
}
#endif
