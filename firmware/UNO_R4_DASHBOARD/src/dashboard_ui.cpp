#include "include/dashboard_ui.h"

#include "include/auto_tft_ui.h"
#include "include/dashboard_timing.h"
#include "include/manual_tft_ui.h"
#include "include/st7796_lcd.h"
#include <string.h>

#if defined(DASHBOARD_HOST)
extern "C" void dashboard_host_publish_ui_state();
#define PUBLISH_HOST_UI_STATE() dashboard_host_publish_ui_state()
#else
#define PUBLISH_HOST_UI_STATE() ((void)0)
#endif

namespace
{

static constexpr uint8_t TRANSITION_ROWS_PER_SERVICE = 4U;

static DashboardValues latest_values = {};
static AutoDashboardValues latest_auto = {};
static bool latest_can_lost = true;
static bool initialized = false;
static bool displayed_auto = false;
static bool desired_auto = false;
static bool transition_active = false;
// Rows [auto_boundary, 319] currently contain AUTO; rows above are MANUAL.
static uint16_t auto_boundary = St7796Lcd::HEIGHT;
static uint8_t latest_input_alive = 0U;
static char swipe_page = '\0';

struct FontGlyph { char character; uint8_t columns[5]; };
static const FontGlyph font5x7[] = {
    {' ', {0x00,0x00,0x00,0x00,0x00}},
    {'!', {0x00,0x00,0x5F,0x00,0x00}},
    {'%', {0x63,0x13,0x08,0x64,0x63}},
    {'+', {0x08,0x08,0x3E,0x08,0x08}},
    {'-', {0x08,0x08,0x08,0x08,0x08}},
    {'.', {0x00,0x60,0x60,0x00,0x00}},
    {'/', {0x20,0x10,0x08,0x04,0x02}},
    {'0', {0x3E,0x51,0x49,0x45,0x3E}},
    {'1', {0x00,0x42,0x7F,0x40,0x00}},
    {'2', {0x42,0x61,0x51,0x49,0x46}},
    {'3', {0x21,0x41,0x45,0x4B,0x31}},
    {'4', {0x18,0x14,0x12,0x7F,0x10}},
    {'5', {0x27,0x45,0x45,0x45,0x39}},
    {'6', {0x3C,0x4A,0x49,0x49,0x30}},
    {'7', {0x01,0x71,0x09,0x05,0x03}},
    {'8', {0x36,0x49,0x49,0x49,0x36}},
    {'9', {0x06,0x49,0x49,0x29,0x1E}},
    {':', {0x00,0x36,0x36,0x00,0x00}},
    {'?', {0x02,0x01,0x51,0x09,0x06}},
    {'A', {0x7E,0x11,0x11,0x11,0x7E}},
    {'B', {0x7F,0x49,0x49,0x49,0x36}},
    {'C', {0x3E,0x41,0x41,0x41,0x22}},
    {'D', {0x7F,0x41,0x41,0x22,0x1C}},
    {'E', {0x7F,0x49,0x49,0x49,0x41}},
    {'F', {0x7F,0x09,0x09,0x09,0x01}},
    {'G', {0x3E,0x41,0x49,0x49,0x7A}},
    {'H', {0x7F,0x08,0x08,0x08,0x7F}},
    {'I', {0x00,0x41,0x7F,0x41,0x00}},
    {'J', {0x20,0x40,0x41,0x3F,0x01}},
    {'K', {0x7F,0x08,0x14,0x22,0x41}},
    {'L', {0x7F,0x40,0x40,0x40,0x40}},
    {'M', {0x7F,0x02,0x0C,0x02,0x7F}},
    {'N', {0x7F,0x04,0x08,0x10,0x7F}},
    {'O', {0x3E,0x41,0x41,0x41,0x3E}},
    {'P', {0x7F,0x09,0x09,0x09,0x06}},
    {'Q', {0x3E,0x41,0x51,0x21,0x5E}},
    {'R', {0x7F,0x09,0x19,0x29,0x46}},
    {'S', {0x46,0x49,0x49,0x49,0x31}},
    {'T', {0x01,0x01,0x7F,0x01,0x01}},
    {'U', {0x3F,0x40,0x40,0x40,0x3F}},
    {'V', {0x1F,0x20,0x40,0x20,0x1F}},
    {'W', {0x7F,0x20,0x18,0x20,0x7F}},
    {'X', {0x63,0x14,0x08,0x14,0x63}},
    {'Y', {0x03,0x04,0x78,0x04,0x03}},
    {'Z', {0x61,0x51,0x49,0x45,0x43}},
};

void drawSwipeText(uint16_t x, uint16_t y, const char *text, uint8_t scale)
{
  for (; *text; ++text, x += 6U * scale)
  {
    for (const FontGlyph &glyph : font5x7)
    {
      if (glyph.character != *text) continue;
      for (uint8_t column = 0; column < 5; ++column)
        for (uint8_t row = 0; row < 7; ++row)
          if (glyph.columns[column] & (1U << row))
            St7796Lcd::fillRect(x + column * scale, y + row * scale,
                              scale, scale, St7796Lcd::COLOR_WHITE);
      break;
    }
  }
}

uint16_t urgentKey(const DashboardValues &values,
                   const AutoDashboardValues &auto_values,
                   bool dashboard_can_lost)
{
  return (dashboard_can_lost ? 1U : 0U) |
      ((values.status_flags & STATUS_ESTOP_ACTIVE) != 0U ? 2U : 0U) |
      ((uint16_t)auto_values.steer_state << 2U);
}

bool flagIsSet(uint8_t value, uint8_t flag)
{
  return (value & flag) != 0U;
}

bool urgentDisplayState(const DashboardValues &values,
                        const AutoDashboardValues &auto_values,
                        bool dashboard_can_lost)
{
  if (dashboard_can_lost || flagIsSet(values.status_flags, STATUS_ESTOP_ACTIVE))
    return true;
  switch (auto_values.steer_state)
  {
    case AUTO_STEER_STATUS_LOST:
    case AUTO_STEER_MOTOR_FAULT:
    case AUTO_STEER_CAN_LOST:
    case AUTO_STEER_POS_LOST:
    case AUTO_STEER_CMD_LOST:
    case AUTO_STEER_TRACK_FAULT:
      return true;
    default:
      return false;
  }
}

void renderTransitionBand(uint16_t y0, uint16_t y1, bool auto_page, uint32_t now_ms)
{
  if (auto_page)
    AutoTftUi::renderPreparedRows(y0, y1, now_ms);
  else
    ManualTftUi::renderPreparedRows(y0, y1);
}

void finishTransition(uint32_t now_ms)
{
  transition_active = false;
  displayed_auto = desired_auto;
  PUBLISH_HOST_UI_STATE();
  DashboardTiming::beginSegment(
      now_ms, displayed_auto ? DashboardTiming::Page::Auto
                             : DashboardTiming::Page::Manual);
  DashboardTiming::modeEvent(
      true, displayed_auto, auto_boundary, latest_input_alive,
      latest_values.status_flags, now_ms);
  if (displayed_auto)
  {
    AutoTftUi::synchronize(now_ms);
  }
  else
  {
    ManualTftUi::synchronize();
  }
}

}  // namespace

namespace DashboardUi
{

void begin(const DashboardValues &values,
           const AutoDashboardValues &auto_values,
           bool dashboard_can_lost)
{
  swipe_page = '\0';
  latest_values = values;
  latest_auto = auto_values;
  latest_can_lost = dashboard_can_lost;
  latest_input_alive = 0U;
  displayed_auto =
      !dashboard_can_lost && flagIsSet(values.status_flags, STATUS_VEHICLE_VALID) &&
      flagIsSet(values.status_flags, STATUS_MODE_AUTO);
  desired_auto = displayed_auto;
  transition_active = false;
  auto_boundary = displayed_auto ? 0U : St7796Lcd::HEIGHT;

  DashboardTiming::beginSegment(
      millis(), displayed_auto ? DashboardTiming::Page::Auto
                               : DashboardTiming::Page::Manual);
  St7796Lcd::setBacklight(false);
  ManualTftUi::observe(values, dashboard_can_lost, millis());
  if (displayed_auto)
  {
    AutoTftUi::begin(values, auto_values, dashboard_can_lost, millis());
  }
  else
  {
    ManualTftUi::begin(values, dashboard_can_lost);
  }
  initialized = true;
  St7796Lcd::setBacklight(true);
}

void update(const DashboardValues &values,
            const AutoDashboardValues &auto_values,
            bool dashboard_can_lost,
            uint8_t input_alive)
{
  if (swipe_page != '\0')
  {
    latest_values = values;
    latest_auto = auto_values;
    latest_can_lost = dashboard_can_lost;
    latest_input_alive = input_alive;
    return;
  }
  if (!initialized)
  {
    begin(values, auto_values, dashboard_can_lost);
    return;
  }
  const uint32_t now_ms = millis();
  const bool urgent_change =
      urgentDisplayState(values, auto_values, dashboard_can_lost) &&
      urgentKey(values, auto_values, dashboard_can_lost) !=
          urgentKey(latest_values, latest_auto, latest_can_lost);
  latest_values = values;
  latest_auto = auto_values;
  latest_can_lost = dashboard_can_lost;
  latest_input_alive = input_alive;
  ManualTftUi::observe(values, dashboard_can_lost, now_ms);
  AutoTftUi::queueUpdate(
      values, auto_values, dashboard_can_lost, now_ms, urgent_change);
  ManualTftUi::update(values, dashboard_can_lost, urgent_change);

  if (!dashboard_can_lost && flagIsSet(values.status_flags, STATUS_VEHICLE_VALID))
  {
    const bool requested_auto = flagIsSet(values.status_flags, STATUS_MODE_AUTO);
    if (requested_auto != desired_auto)
    {
      DashboardTiming::modeEvent(
          false, requested_auto, auto_boundary, input_alive,
          values.status_flags, now_ms);
      desired_auto = requested_auto;
      transition_active = true;
    }
  }
  if (transition_active || displayed_auto) return;
}

void service(uint32_t now_ms)
{
  DashboardTiming::ServiceScope timing_scope;
  if (!initialized) return;
  if (swipe_page != '\0') return;
  if (transition_active)
  {
    const uint32_t service_started_us = micros();
    uint8_t rendered = 0U;
    while (transition_active && rendered < TRANSITION_ROWS_PER_SERVICE)
    {
      if (desired_auto)
      {
        if (auto_boundary == 0U)
        {
          finishTransition(millis());
          break;
        }
        const uint16_t y = (uint16_t)(auto_boundary - 1U);
        renderTransitionBand(y, y, true, now_ms);
        auto_boundary = y;
        PUBLISH_HOST_UI_STATE();
      }
      else
      {
        if (auto_boundary >= St7796Lcd::HEIGHT)
        {
          finishTransition(millis());
          break;
        }
        const uint16_t y = auto_boundary;
        renderTransitionBand(y, y, false, now_ms);
        auto_boundary = (uint16_t)(y + 1U);
        PUBLISH_HOST_UI_STATE();
      }
      rendered++;
      if ((uint32_t)(micros() - service_started_us) >= 2000U) break;
    }
    if (transition_active &&
        ((desired_auto && auto_boundary == 0U) ||
         (!desired_auto && auto_boundary >= St7796Lcd::HEIGHT)))
      finishTransition(millis());
    return;
  }
  if (displayed_auto) AutoTftUi::service(now_ms);
  else ManualTftUi::service(now_ms);
}

void redraw()
{
  if (!initialized) return;
  if (displayed_auto)
    AutoTftUi::begin(latest_values, latest_auto, latest_can_lost, millis());
  else
    ManualTftUi::begin(latest_values, latest_can_lost);
}

void observeAuto(const AutoDashboardValues &auto_values, uint32_t now_ms)
{
  AutoTftUi::observe(auto_values, now_ms);
}

void resetState()
{
  latest_values = {};
  latest_auto = {};
  latest_can_lost = true;
  initialized = false;
  displayed_auto = false;
  desired_auto = false;
  transition_active = false;
  auto_boundary = St7796Lcd::HEIGHT;
  latest_input_alive = 0U;
  swipe_page = '\0';
  ManualTftUi::resetState();
  AutoTftUi::resetState();
  PUBLISH_HOST_UI_STATE();
}

bool isTransitionActive() { return transition_active; }
bool isAutoDisplayed() { return displayed_auto; }
uint16_t transitionBoundary() { return auto_boundary; }
const DashboardValues &displayedValues() { return latest_values; }
const AutoDashboardValues &displayedAutoValues() { return latest_auto; }
bool canLost() { return latest_can_lost; }
const char *pageName()
{
  if (swipe_page == 'L') return "SWIPE_L";
  if (swipe_page == 'R') return "SWIPE_R";
  if (swipe_page != '\0') return "SWIPE_WAIT";
  return displayed_auto ? "AUTO_GT3" : "MANUAL_TFT";
}

void showSwipeTest(char result)
{
  if (result == swipe_page) return;
  if (result == '\0')
  {
    begin(latest_values, latest_auto, latest_can_lost);
    return;
  }
  swipe_page = result;
  St7796Lcd::fillScreen(St7796Lcd::COLOR_BLACK);
  const char *hint = result == 'L' ? "SWIPE LEFT TO DASHBOARD" :
                     result == 'R' ? "SWIPE RIGHT TO DASHBOARD" : "SWIPE LEFT OR RIGHT";
  drawSwipeText(uint16_t((St7796Lcd::WIDTH - strlen(hint) * 12U) / 2U), 24U, hint, 2U);
  const char letter[] = {result, '\0'};
  drawSwipeText(180U, 76U, letter, 24U);
  PUBLISH_HOST_UI_STATE();
}

}  // namespace DashboardUi
