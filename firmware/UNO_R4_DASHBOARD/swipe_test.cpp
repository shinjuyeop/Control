#include "swipe_test.h"
#include "src/include/dashboard_ui.h"
#include "ft6336_touch.h"

namespace
{
TouchNavigation::Navigator navigator;
uint32_t last_sample = 0;
bool physical_contact = false;
TouchNavigation::TouchSample physical{
  TouchNavigation::Phase::UP, TouchNavigation::Source::PHYSICAL, 0, 0};

void apply(const TouchNavigation::TouchSample &sample, uint32_t now_ms)
{
  if (navigator.accept(sample, now_ms))
    DashboardUi::showSwipeTest(navigator.page() < 0 ? 'L' : navigator.page() > 0 ? 'R' : '\0');
}
}

// Keep the legacy Serial SWIPE_TEST switch; this is now only the hardware adapter.
namespace SwipeTest
{
void setEnabled(bool value)
{
  navigator.setEnabled(value);
  physical_contact = true;  // Re-enable only after all physical fingers release.
  last_sample = Ft6336Touch::state().samples;
  DashboardUi::showSwipeTest('\0');
}

void virtualTouch(const TouchNavigation::TouchSample &sample, uint32_t now_ms)
{
  apply(sample, now_ms);
}

void cancelVirtual() { navigator.cancelVirtual(); }

void poll(uint32_t now_ms)
{
  navigator.poll(now_ms);
  const auto &t = Ft6336Touch::state();
  if (t.samples == last_sample) return;
  last_sample = t.samples;
  if (!t.valid || t.count > 1)
  {
    physical.phase = TouchNavigation::Phase::CANCEL;
    physical_contact = true;
  }
  else if (t.count == 0)
  {
    physical.phase = TouchNavigation::Phase::UP;
    physical_contact = false;
  }
  else
  {
    const auto &point = t.points[0];
    physical.phase = physical_contact ? TouchNavigation::Phase::MOVE : TouchNavigation::Phase::DOWN;
    physical_contact = true;
    // Validated landscape mapping. Raw 320 x 480 never reaches the recognizer.
    physical.x = int16_t(point.y);
    physical.y = 319 - int16_t(point.x);
    physical.finger = point.id;
  }
  apply(physical, now_ms);
}
}
