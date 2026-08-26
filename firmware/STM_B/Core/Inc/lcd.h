#ifndef LCD_H
#define LCD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
  uint16_t width;
  uint16_t height;
  uint8_t rotation;
} LCD_Handle_t;

#define LCD_COLOR_BLACK 0x0000U
#define LCD_COLOR_WHITE 0xFFFFU
#define LCD_COLOR_RED   0xF800U
#define LCD_COLOR_GREEN 0x07E0U

#define LCD_PORTRAIT_WIDTH  320U
#define LCD_PORTRAIT_HEIGHT 480U
#define LCD_LANDSCAPE_WIDTH  480U
#define LCD_LANDSCAPE_HEIGHT 320U

#define LCD_ROTATION_PORTRAIT  0U
#define LCD_ROTATION_LANDSCAPE 1U

/* Portrait and landscape geometries are supported. False means that the
 * selected geometry is invalid or an SPI transfer failed during setup. */
bool LCD_Init(const LCD_Handle_t *lcd);

/* Re-reads the display status register (RDDST) and compares it against the
 * baseline captured right after the last successful LCD_Init(). Returns
 * true only when a successful read differs from that baseline, meaning the
 * controller state was corrupted (e.g. by a ground bounce) since init. */
bool LCD_IsFaulted(void);

void LCD_FillScreen(uint16_t color);
void LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void LCD_DrawChar(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg);
void LCD_DrawString(uint16_t x, uint16_t y, const char *text, uint16_t fg, uint16_t bg);

#ifdef __cplusplus
}
#endif

#endif /* LCD_H */
