#include "include/dashboard_ui.h"

#include "include/auto_tft_ui.h"
#include "include/dashboard_timing.h"
#include "include/manual_tft_ui.h"
#include "include/st7796_lcd.h"

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
const char *pageName() { return displayed_auto ? "AUTO_GT3" : "MANUAL_TFT"; }

}  // namespace DashboardUi
