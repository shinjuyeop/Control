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
#define LCD_COLOR_BLUE  0x045FU
#define LCD_COLOR_ORANGE 0xFC60U
#define LCD_COLOR_LIGHT_GRAY 0xE71CU
#define LCD_COLOR_DARK_GRAY  0x4208U

#define LCD_PORTRAIT_WIDTH  320U
#define LCD_PORTRAIT_HEIGHT 480U
#define LCD_LANDSCAPE_WIDTH  480U
#define LCD_LANDSCAPE_HEIGHT 320U
#define LCD_COMPOSE_BAND_MAX_ROWS 4U

#define LCD_ROTATION_PORTRAIT  0U
#define LCD_ROTATION_LANDSCAPE 1U

typedef enum
{
  LCD_ERROR_NONE = 0U,
  LCD_ERROR_COMMAND_TX = 1U,
  LCD_ERROR_DATA_TX = 2U,
  LCD_ERROR_DMA_START = 3U,
  LCD_ERROR_DMA_TIMEOUT = 4U,
  LCD_ERROR_DMA_CALLBACK = 5U,
  LCD_ERROR_READ_COMMAND = 6U,
  LCD_ERROR_READ_DATA = 7U,
  LCD_ERROR_HEALTH_STATUS = 8U
} LCD_Error_t;

extern volatile uint32_t LCD_ERROR_COUNT;
extern volatile uint32_t LCD_LAST_ERROR;

/* Portrait and landscape geometries are supported. False means that the
 * selected geometry is invalid or an SPI transfer failed during setup. */
bool LCD_Init(const LCD_Handle_t *lcd);
void LCD_SetBacklight(bool enabled);
bool LCD_ReadPowerMode(uint8_t *power_mode);
bool LCD_IsHealthy(void);
/* Compose final pixels in RAM and publish one complete horizontal band. */
bool LCD_BeginComposeBand(uint16_t y, uint16_t height);
bool LCD_EndComposeBand(void);
void LCD_CancelComposeBand(void);
bool LCD_FillScreen(uint16_t color);
bool LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
bool LCD_DrawRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                  uint16_t thickness, uint16_t color);
bool LCD_DrawCharScaled(uint16_t x, uint16_t y, char c, uint8_t scale,
                        uint16_t fg, uint16_t bg);
bool LCD_DrawStringScaled(uint16_t x, uint16_t y, const char *text,
                          uint8_t scale, uint16_t fg, uint16_t bg);
bool LCD_DrawChar(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg);
bool LCD_DrawString(uint16_t x, uint16_t y, const char *text,
                    uint16_t fg, uint16_t bg);

#ifdef __cplusplus
}
#endif

#endif /* LCD_H */
