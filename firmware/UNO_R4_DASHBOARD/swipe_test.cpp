#include "swipe_test.h"
#include "dashboard_ui.h"
#include "ft6336_touch.h"
#include "st7796_lcd.h"
#include <stdlib.h>

namespace
{
bool enabled = false;
bool tracking = false;
bool wait_release = false;
int page = 0;  // -1=L, 0=live dashboard, 1=R.
uint8_t finger_id = 0;
uint32_t last_sample = 0;
int start_x = 0, start_y = 0, last_x = 0, last_y = 0;
// A horizontal move of one eighth of the screen distinguishes a swipe from a tap.
constexpr int MIN_DISTANCE = St7796Lcd::WIDTH / 8;
}

namespace SwipeTest
{
void setEnabled(bool value)
{
  enabled = value;
  tracking = false;
  wait_release = true;
  page = 0;
  last_sample = Ft6336Touch::state().samples;
  DashboardUi::showSwipeTest('\0');
}

void poll()
{
  const auto &t = Ft6336Touch::state();
  if (!enabled || t.samples == last_sample) return;
  last_sample = t.samples;
  if (!t.valid || t.count > 1)
  {
    tracking = false;
    wait_release = true;
    return;
  }
  if (t.count == 0)
  {
    if (tracking)
    {
      const int dx = last_x - start_x;
      const int dy = last_y - start_y;
      if (abs(dx) >= MIN_DISTANCE && abs(dx) > abs(dy))
      {
        // Drag the page with the finger: right reveals the page on the left.
        const int next_page = page + (dx > 0 ? -1 : 1);
        if (next_page >= -1 && next_page <= 1)
        {
          page = next_page;
          DashboardUi::showSwipeTest(page < 0 ? 'L' : page > 0 ? 'R' : '\0');
        }
      }
    }
    tracking = false;
    wait_release = false;
    return;
  }
  if (wait_release) return;
  const auto &point = t.points[0];
  // Mapping established by the physical corner test on this landscape display.
  last_x = point.y;
  last_y = int(St7796Lcd::HEIGHT) - 1 - int(point.x);
  if (!tracking)
  {
    start_x = last_x;
    start_y = last_y;
    finger_id = point.id;
    tracking = true;
  }
  else if (point.id != finger_id)
  {
    tracking = false;
    wait_release = true;
  }
}
}
