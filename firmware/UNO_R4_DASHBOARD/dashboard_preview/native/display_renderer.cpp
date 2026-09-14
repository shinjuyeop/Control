// Render the current firmware UI from the values reported by the UNO.
#include "host_runtime.h"
#include "dashboard_ui.h"
#include "auto_tft_ui.h"
#include "manual_tft_ui.h"
#include "st7796_lcd.h"

#include <array>
#include <cstdint>

extern "C" void dashboard_host_publish_ui_state() {}

extern "C" void render_auto_dashboard(const int32_t *native_values,
                                 const int32_t *auto_values,
                                 int can_lost,
                                 int auto_page,
                                 uint8_t *rgb)
{
  const DashboardValues native{
      uint8_t(native_values[0]), uint8_t(native_values[1]),
      uint8_t(native_values[2]), uint8_t(native_values[3]),
      int16_t(native_values[4]), uint16_t(native_values[5]),
      int16_t(native_values[6]), int16_t(native_values[7]),
      int16_t(native_values[8]), int16_t(native_values[9]),
      uint32_t(native_values[10])};
  const AutoDashboardValues automatic{
      uint8_t(auto_values[0]), uint8_t(auto_values[1]),
      uint8_t(auto_values[2]), uint8_t(auto_values[3]),
      uint8_t(auto_values[4]), uint8_t(auto_values[5]),
      uint8_t(auto_values[6]), uint8_t(auto_values[7]),
      uint8_t(auto_values[8]), uint8_t(auto_values[9]),
      uint8_t(auto_values[10]), uint16_t(auto_values[11]),
      uint16_t(auto_values[12]), int16_t(auto_values[13]),
      int32_t(auto_values[14]), int32_t(auto_values[15]),
      int16_t(auto_values[16])};

  DashboardHostRuntime::reset();
  DashboardHostRuntime::setAllowedTimeUs(60000000U);
  St7796Lcd::begin();
  DashboardUi::resetState();
  // The UNO-reported page remains authoritative during mode transitions and loss.
  if (auto_page)
    AutoTftUi::begin(native, automatic, can_lost != 0, millis());
  else
    ManualTftUi::begin(native, can_lost != 0);
  St7796Lcd::setBacklight(true);

  std::array<uint16_t, St7796Lcd::WIDTH * St7796Lcd::HEIGHT> pixels{};
  DashboardHostRuntime::copyFramebuffer(pixels.data());
  for (std::size_t index = 0; index < pixels.size(); ++index)
  {
    const uint16_t color = pixels[index];
    rgb[index * 3U] = uint8_t(((color >> 11U) & 31U) * 255U / 31U);
    rgb[index * 3U + 1U] = uint8_t(((color >> 5U) & 63U) * 255U / 63U);
    rgb[index * 3U + 2U] = uint8_t((color & 31U) * 255U / 31U);
  }
}

// Keep main's three-argument entrypoint for already installed Console clients.
extern "C" void render_dashboard(const int32_t *native_values,
                                 int can_lost,
                                 uint8_t *rgb)
{
  const int32_t unavailable_auto[] = {
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      AUTO_HEADING_INVALID, AUTO_HEADING_INVALID, AUTO_I16_INVALID,
      AUTO_I32_INVALID, AUTO_I32_INVALID, AUTO_I16_INVALID};
  render_auto_dashboard(native_values, unavailable_auto, can_lost,
                        (native_values[0] & STATUS_MODE_AUTO) != 0, rgb);
}

extern "C" void render_swipe_test(int result, uint8_t *rgb)
{
  DashboardUi::resetState();
  DashboardHostRuntime::reset();
  DashboardHostRuntime::setAllowedTimeUs(60000000U);
  St7796Lcd::begin();
  DashboardUi::showSwipeTest(result == 1 ? 'L' : result == 2 ? 'R' : '-');
  St7796Lcd::setBacklight(true);

  std::array<uint16_t, St7796Lcd::WIDTH * St7796Lcd::HEIGHT> pixels{};
  DashboardHostRuntime::copyFramebuffer(pixels.data());
  for (std::size_t index = 0; index < pixels.size(); ++index)
  {
    const uint16_t color = pixels[index];
    rgb[index * 3U] = uint8_t(((color >> 11U) & 31U) * 255U / 31U);
    rgb[index * 3U + 1U] = uint8_t(((color >> 5U) & 63U) * 255U / 63U);
    rgb[index * 3U + 2U] = uint8_t((color & 31U) * 255U / 31U);
  }
}
