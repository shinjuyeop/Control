#ifndef TOUCH_NAVIGATION_H
#define TOUCH_NAVIGATION_H

#include <stdint.h>

namespace TouchNavigation
{
constexpr int WIDTH = 480, HEIGHT = 320;
constexpr uint32_t PHYSICAL_GUARD_MS = 150;
constexpr uint32_t VIRTUAL_TIMEOUT_MS = 1000;
enum class Phase : uint8_t { DOWN, MOVE, UP, CANCEL };
enum class Source : uint8_t { PHYSICAL, VIRTUAL };

struct TouchSample
{
  Phase phase;
  Source source;
  int16_t x, y;  // LCD screen coordinates, independent of the input device.
  uint8_t finger = 0;
  uint8_t contacts = 1;
};

// Returns a single page step only at release. No Arduino or display dependency.
class Gesture
{
public:
  int8_t accept(const TouchSample &sample);
  void cancel() { tracking_ = false; }
private:
  bool tracking_ = false;
  TouchSample start_{};
};

class Navigator
{
public:
  void setEnabled(bool enabled);
  bool accept(const TouchSample &sample, uint32_t now_ms);
  void cancelVirtual();
  void poll(uint32_t now_ms);
  int8_t page() const { return page_; }
private:
  Gesture gesture_;
  bool enabled_ = false;
  bool physical_busy_ = true;  // Wait for the first valid physical release.
  bool physical_guard_ = false;
  bool virtual_active_ = false;
  int8_t page_ = 0;
  uint32_t physical_release_ms_ = 0, virtual_sample_ms_ = 0;
};

// Exact, bounded ASCII grammar: VT D|M|U x y, or VT C. No allocation.
bool parseVirtualTouch(const char *line, TouchSample &sample);
}
#endif
