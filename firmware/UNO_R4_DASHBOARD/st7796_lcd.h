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
void drawRect(uint16_t x,
              uint16_t y,
              uint16_t width,
              uint16_t height,
              uint16_t thickness,
              uint16_t color);
void drawCharScaled(uint16_t x,
                    uint16_t y,
                    char character,
                    uint8_t scale,
                    uint16_t foreground,
                    uint16_t background);
void drawStringScaled(uint16_t x,
                      uint16_t y,
                      const char *text,
                      uint8_t scale,
                      uint16_t foreground,
                      uint16_t background);

}  // namespace St7796Lcd

#endif  // ST7796_LCD_H
