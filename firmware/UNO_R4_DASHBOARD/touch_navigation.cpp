#include "touch_navigation.h"
#include <stdlib.h>

namespace TouchNavigation
{
int8_t Gesture::accept(const TouchSample &s)
{
  if (s.phase == Phase::CANCEL || s.contacts != 1 ||
      s.x < 0 || s.x >= WIDTH || s.y < 0 || s.y >= HEIGHT)
  {
    cancel();
    return 0;
  }
  if (s.phase == Phase::DOWN)
  {
    if (tracking_) { cancel(); return 0; }
    start_ = s;
    tracking_ = true;
    return 0;
  }
  if (!tracking_) return 0;
  if (s.source != start_.source || s.finger != start_.finger)
  {
    cancel();
    return 0;
  }
  if (s.phase != Phase::UP) return 0;
  cancel();
  const int dx = s.x - start_.x, dy = s.y - start_.y;
  // Drag right reveals L; drag left reveals R. Exactly 60 px qualifies.
  return abs(dx) >= 60 && abs(dx) > abs(dy) ? (dx > 0 ? -1 : 1) : 0;
}

void Navigator::setEnabled(bool enabled)
{
  enabled_ = enabled;
  gesture_.cancel();
  virtual_active_ = false;
  page_ = 0;
}

void Navigator::cancelVirtual()
{
  if (virtual_active_) gesture_.cancel();
  virtual_active_ = false;
}

void Navigator::poll(uint32_t now_ms)
{
  if (virtual_active_ && uint32_t(now_ms - virtual_sample_ms_) >= VIRTUAL_TIMEOUT_MS)
    cancelVirtual();
}

bool Navigator::accept(const TouchSample &s, uint32_t now_ms)
{
  poll(now_ms);
  if (s.source == Source::PHYSICAL)
  {
    if (s.phase == Phase::UP)
    {
      if (!physical_busy_) return false;  // Idle polls must not release a virtual drag.
      physical_busy_ = false;
      physical_guard_ = true;
      physical_release_ms_ = now_ms;
    }
    else
    {
      physical_busy_ = true;
      cancelVirtual();
    }
  }
  else
  {
    if (s.phase == Phase::CANCEL) { cancelVirtual(); return false; }
    if (physical_busy_ || (physical_guard_ &&
        uint32_t(now_ms - physical_release_ms_) < PHYSICAL_GUARD_MS)) return false;
    if (s.phase == Phase::DOWN) virtual_active_ = true;
    else if (!virtual_active_) return false;
    virtual_sample_ms_ = now_ms;
    if (s.phase == Phase::UP) virtual_active_ = false;
  }
  if (!enabled_) return false;
  const int next = page_ + gesture_.accept(s);
  if (next == page_ || next < -1 || next > 1) return false;
  page_ = int8_t(next);
  return true;
}

namespace
{
bool coordinate(const char *&p, int16_t &value)
{
  unsigned number = 0, digits = 0;
  while (*p >= '0' && *p <= '9')
  {
    if (++digits > 3) return false;
    number = number * 10 + unsigned(*p++ - '0');
  }
  value = int16_t(number);
  return digits != 0;
}
}

bool parseVirtualTouch(const char *line, TouchSample &sample)
{
  const char *p = line;
  if (*p++ != 'V' || *p++ != 'T' || *p++ != ' ') return false;
  TouchSample s{Phase::CANCEL, Source::VIRTUAL, 0, 0};
  if (*p == 'C' && p[1] == '\0') { sample = s; return true; }
  if (*p == 'D') s.phase = Phase::DOWN;
  else if (*p == 'M') s.phase = Phase::MOVE;
  else if (*p == 'U') s.phase = Phase::UP;
  else return false;
  ++p;
  if (*p++ != ' ' || !coordinate(p, s.x) || *p++ != ' ' ||
      !coordinate(p, s.y) || *p != '\0' || s.x >= WIDTH || s.y >= HEIGHT) return false;
  sample = s;
  return true;
}
}
