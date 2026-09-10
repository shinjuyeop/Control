#ifndef FT6336_TOUCH_H
#define FT6336_TOUCH_H

#include <stdint.h>

namespace Ft6336Touch
{
struct Point
{
  uint16_t x;
  uint16_t y;
  uint8_t id;
  uint8_t event;  // FT6336U: 0=down, 1=up, 2=contact.
};

struct State
{
  bool identified;
  bool valid;
  uint8_t vendor;
  uint8_t chip_high;
  uint8_t chip_mid;
  uint8_t chip_low;
  uint8_t firmware;
  uint8_t count;
  uint8_t irq_level;
  uint32_t irq_edges;
  uint32_t samples;
  uint32_t errors;
  uint32_t sample_ms;
  Point points[2];
};

void begin();
void poll(uint32_t now_ms);
const State &state();
}
#endif
