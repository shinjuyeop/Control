#ifndef ST7796_LCD_H
#define ST7796_LCD_H

#include <Arduino.h>

namespace St7796Lcd
{

static constexpr uint16_t WIDTH = 480U;
static constexpr uint16_t HEIGHT = 320U;

static constexpr uint16_t COLOR_BLACK = 0x0000U;
static constexpr uint16_t COLOR_WHITE = 0xFFFFU;
static constexpr uint16_t COLOR_RED = 0xF800U;
static constexpr uint16_t COLOR_GREEN = 0x07E0U;
static constexpr uint16_t COLOR_BLUE = 0x045FU;
static constexpr uint16_t COLOR_ORANGE = 0xFC60U;
static constexpr uint16_t COLOR_LIGHT_GRAY = 0xE71CU;
static constexpr uint16_t COLOR_DARK_GRAY = 0x4208U;

void begin();
void setBacklight(bool enabled);
void fillRect(uint16_t x,
              uint16_t y,
              uint16_t width,
              uint16_t height,
              uint16_t color);
void fillScreen(uint16_t color);
void beginPixelStream();
void beginPixelStream(uint16_t x0,
                      uint16_t y0,
                      uint16_t x1,
                      uint16_t y1);
void writePixelRow(const uint16_t *pixels, size_t count);
void endPixelStream();
#if defined(DASHBOARD_BENCH_INPUT) && \
    defined(DASHBOARD_TIMING_TRANSPORT_PROFILE)
void runTransportCalibration();
#endif

}  // namespace St7796Lcd

#endif  // ST7796_LCD_H
