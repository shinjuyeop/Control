#include "lcd.h"

#include <stddef.h>

#include "main.h"
#include "spi.h"

#define LCD_CMD_SLPOUT  0x11U
#define LCD_CMD_NORON   0x13U
#define LCD_CMD_DISPON  0x29U
#define LCD_CMD_CASET   0x2AU
#define LCD_CMD_RASET   0x2BU
#define LCD_CMD_RAMWR   0x2CU
#define LCD_CMD_MADCTL  0x36U
#define LCD_CMD_COLMOD  0x3AU

#define LCD_SPI_TIMEOUT_MS 100U
#define LCD_CHAR_W 5U
#define LCD_CHAR_H 7U
#define LCD_CHAR_SCALE 3U
#define LCD_CHAR_ADVANCE ((LCD_CHAR_W + 1U) * LCD_CHAR_SCALE)
#define LCD_INIT_MAX_DATA 14U

#define LCD_MADCTL_BGR 0x08U
#define LCD_MADCTL_MV  0x20U
#define LCD_MADCTL_MX  0x40U

typedef struct
{
  uint8_t command;
  uint8_t length;
  uint8_t data[LCD_INIT_MAX_DATA];
} LCD_InitCommand_t;

static LCD_Handle_t active_lcd = {
    LCD_LANDSCAPE_WIDTH,
    LCD_LANDSCAPE_HEIGHT,
    LCD_ROTATION_LANDSCAPE};

/* MSP4030 vendor sequence for its TN panel and ST7796S controller.
 * The power, gate and gamma values are panel-specific. */
static const LCD_InitCommand_t msp4030_init[] = {
    {0xF0U, 1U, {0xC3U}},
    {0xF0U, 1U, {0x96U}},
    {LCD_CMD_MADCTL, 1U, {0x48U}},
    {LCD_CMD_COLMOD, 1U, {0x05U}},
    {0xB0U, 1U, {0x80U}},
    {0xB6U, 2U, {0x00U, 0x02U}},
    {0xB5U, 4U, {0x02U, 0x03U, 0x00U, 0x04U}},
    {0xB1U, 2U, {0x80U, 0x10U}},
    {0xB4U, 1U, {0x00U}},
    {0xB7U, 1U, {0xC6U}},
    {0xC5U, 1U, {0x1CU}},
    {0xE4U, 1U, {0x31U}},
    {0xE8U, 8U, {0x40U, 0x8AU, 0x00U, 0x00U, 0x29U, 0x19U, 0xA5U, 0x33U}},
    {0xC2U, 1U, {0xA7U}},
    {0xE0U, 14U, {0xF0U, 0x09U, 0x13U, 0x12U, 0x12U, 0x2BU, 0x3CU,
                   0x44U, 0x4BU, 0x1BU, 0x18U, 0x17U, 0x1DU, 0x21U}},
    {0xE1U, 14U, {0xF0U, 0x09U, 0x13U, 0x0CU, 0x0DU, 0x27U, 0x3BU,
                   0x44U, 0x4DU, 0x0BU, 0x17U, 0x17U, 0x1DU, 0x21U}},
    {0xF0U, 1U, {0x3CU}},
    {0xF0U, 1U, {0x69U}},
};

static const uint8_t font5x7[][5] = {
    [' '] = {0x00,0x00,0x00,0x00,0x00},
    ['!'] = {0x00,0x00,0x5F,0x00,0x00},
    ['.'] = {0x00,0x60,0x60,0x00,0x00},
    ['/'] = {0x20,0x10,0x08,0x04,0x02},
    ['0'] = {0x3E,0x51,0x49,0x45,0x3E},
    ['1'] = {0x00,0x42,0x7F,0x40,0x00},
    ['2'] = {0x42,0x61,0x51,0x49,0x46},
    ['3'] = {0x21,0x41,0x45,0x4B,0x31},
    ['4'] = {0x18,0x14,0x12,0x7F,0x10},
    ['5'] = {0x27,0x45,0x45,0x45,0x39},
    ['6'] = {0x3C,0x4A,0x49,0x49,0x30},
    ['7'] = {0x01,0x71,0x09,0x05,0x03},
    ['8'] = {0x36,0x49,0x49,0x49,0x36},
    ['9'] = {0x06,0x49,0x49,0x29,0x1E},
    [':'] = {0x00,0x36,0x36,0x00,0x00},
    ['A'] = {0x7E,0x11,0x11,0x11,0x7E},
    ['D'] = {0x7F,0x41,0x41,0x22,0x1C},
    ['E'] = {0x7F,0x49,0x49,0x49,0x41},
    ['F'] = {0x7F,0x09,0x09,0x09,0x01},
    ['G'] = {0x3E,0x41,0x49,0x49,0x7A},
    ['L'] = {0x7F,0x40,0x40,0x40,0x40},
    ['M'] = {0x7F,0x02,0x0C,0x02,0x7F},
    ['N'] = {0x7F,0x04,0x08,0x10,0x7F},
    ['O'] = {0x3E,0x41,0x41,0x41,0x3E},
    ['P'] = {0x7F,0x09,0x09,0x09,0x06},
    ['R'] = {0x7F,0x09,0x19,0x29,0x46},
    ['S'] = {0x46,0x49,0x49,0x49,0x31},
    ['T'] = {0x01,0x01,0x7F,0x01,0x01},
    ['U'] = {0x3F,0x40,0x40,0x40,0x3F},
    ['V'] = {0x1F,0x20,0x40,0x20,0x1F},
    ['W'] = {0x7F,0x20,0x18,0x20,0x7F},
    ['a'] = {0x20,0x54,0x54,0x54,0x78},
    ['d'] = {0x38,0x44,0x44,0x48,0x7F},
    ['e'] = {0x38,0x54,0x54,0x54,0x18},
    ['h'] = {0x7F,0x08,0x04,0x04,0x78},
    ['k'] = {0x7F,0x10,0x28,0x44,0x00},
    ['m'] = {0x7C,0x04,0x18,0x04,0x78},
    ['o'] = {0x38,0x44,0x44,0x44,0x38},
    ['p'] = {0x7C,0x14,0x14,0x14,0x08},
    ['r'] = {0x7C,0x08,0x04,0x04,0x08},
    ['s'] = {0x48,0x54,0x54,0x54,0x20},
};

static void LCD_Select(void)
{
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);
}

static void LCD_Unselect(void)
{
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
}

static bool LCD_WriteCommand(uint8_t command)
{
  HAL_StatusTypeDef status;

  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
  LCD_Select();
  status = HAL_SPI_Transmit(&hspi2, &command, 1U, LCD_SPI_TIMEOUT_MS);
  LCD_Unselect();
  return status == HAL_OK;
}

static bool LCD_WriteData(const uint8_t *data, uint16_t length)
{
  HAL_StatusTypeDef status;

  if ((data == NULL) || (length == 0U))
  {
    return false;
  }

  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
  LCD_Select();
  status = HAL_SPI_Transmit(&hspi2, (uint8_t *)data, length, LCD_SPI_TIMEOUT_MS);
  LCD_Unselect();
  return status == HAL_OK;
}

static bool LCD_WriteCommandData(uint8_t command, const uint8_t *data, uint16_t length)
{
  return LCD_WriteCommand(command) && ((length == 0U) || LCD_WriteData(data, length));
}

static bool LCD_SetAddressWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  uint8_t data[4];

  data[0] = (uint8_t)(x0 >> 8);
  data[1] = (uint8_t)(x0 & 0xFFU);
  data[2] = (uint8_t)(x1 >> 8);
  data[3] = (uint8_t)(x1 & 0xFFU);
  if (!LCD_WriteCommandData(LCD_CMD_CASET, data, sizeof(data)))
  {
    return false;
  }

  data[0] = (uint8_t)(y0 >> 8);
  data[1] = (uint8_t)(y0 & 0xFFU);
  data[2] = (uint8_t)(y1 >> 8);
  data[3] = (uint8_t)(y1 & 0xFFU);
  if (!LCD_WriteCommandData(LCD_CMD_RASET, data, sizeof(data)))
  {
    return false;
  }

  return LCD_WriteCommand(LCD_CMD_RAMWR);
}

bool LCD_Init(const LCD_Handle_t *lcd)
{
  uint8_t madctl;

  if (lcd == NULL)
  {
    return false;
  }

  if ((lcd->rotation == LCD_ROTATION_PORTRAIT) &&
      (lcd->width == LCD_PORTRAIT_WIDTH) &&
      (lcd->height == LCD_PORTRAIT_HEIGHT))
  {
    madctl = LCD_MADCTL_BGR | LCD_MADCTL_MX;
  }
  else if ((lcd->rotation == LCD_ROTATION_LANDSCAPE) &&
           (lcd->width == LCD_LANDSCAPE_WIDTH) &&
           (lcd->height == LCD_LANDSCAPE_HEIGHT))
  {
    /* MSP4030 vendor direction 1: exchange row/column addressing while
     * retaining the panel's BGR color order. */
    madctl = LCD_MADCTL_BGR | LCD_MADCTL_MV;
  }
  else
  {
    return false;
  }
  active_lcd = *lcd;

  HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(100U);
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(50U);

  for (uint32_t i = 0U; i < (sizeof(msp4030_init) / sizeof(msp4030_init[0])); i++)
  {
    const uint8_t *data = msp4030_init[i].data;

    if (msp4030_init[i].command == LCD_CMD_MADCTL)
    {
      data = &madctl;
    }

    if (!LCD_WriteCommandData(msp4030_init[i].command,
                              data,
                              msp4030_init[i].length))
    {
      return false;
    }
  }

  if (!LCD_WriteCommand(LCD_CMD_NORON) || !LCD_WriteCommand(LCD_CMD_SLPOUT))
  {
    return false;
  }
  HAL_Delay(120U);
  if (!LCD_WriteCommand(LCD_CMD_DISPON))
  {
    return false;
  }
  HAL_Delay(20U);
  HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_SET);
  return true;
}

void LCD_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
{
  uint8_t pixels[64];
  uint32_t total;

  if ((x >= active_lcd.width) || (y >= active_lcd.height) || (w == 0U) || (h == 0U))
  {
    return;
  }

  if ((uint32_t)x + w > active_lcd.width)
  {
    w = (uint16_t)(active_lcd.width - x);
  }
  if ((uint32_t)y + h > active_lcd.height)
  {
    h = (uint16_t)(active_lcd.height - y);
  }

  for (uint32_t i = 0U; i < sizeof(pixels); i += 2U)
  {
    pixels[i] = (uint8_t)(color >> 8);
    pixels[i + 1U] = (uint8_t)(color & 0xFFU);
  }

  if (!LCD_SetAddressWindow(x, y, (uint16_t)(x + w - 1U), (uint16_t)(y + h - 1U)))
  {
    return;
  }
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
  LCD_Select();

  total = (uint32_t)w * h;
  while (total > 0U)
  {
    uint16_t chunk_pixels = (total > (sizeof(pixels) / 2U))
                              ? (uint16_t)(sizeof(pixels) / 2U)
                              : (uint16_t)total;
    if (HAL_SPI_Transmit(&hspi2,
                         pixels,
                         (uint16_t)(chunk_pixels * 2U),
                         LCD_SPI_TIMEOUT_MS) != HAL_OK)
    {
      break;
    }
    total -= chunk_pixels;
  }

  LCD_Unselect();
}

void LCD_FillScreen(uint16_t color)
{
  LCD_FillRect(0U, 0U, active_lcd.width, active_lcd.height, color);
}

void LCD_DrawChar(uint16_t x, uint16_t y, char c, uint16_t fg, uint16_t bg)
{
  const uint8_t *glyph;
  uint8_t pixels[LCD_CHAR_ADVANCE * LCD_CHAR_H * LCD_CHAR_SCALE * 2U];
  uint32_t index = 0U;

  if ((uint8_t)c >= (sizeof(font5x7) / sizeof(font5x7[0])))
  {
    c = ' ';
  }

  glyph = font5x7[(uint8_t)c];
  for (uint8_t row = 0U; row < LCD_CHAR_H; row++)
  {
    for (uint8_t row_scale = 0U; row_scale < LCD_CHAR_SCALE; row_scale++)
    {
      for (uint8_t column = 0U; column < (LCD_CHAR_W + 1U); column++)
      {
        uint16_t color = bg;
        if ((column < LCD_CHAR_W) && ((glyph[column] & (1U << row)) != 0U))
        {
          color = fg;
        }

        for (uint8_t column_scale = 0U; column_scale < LCD_CHAR_SCALE; column_scale++)
        {
          pixels[index++] = (uint8_t)(color >> 8);
          pixels[index++] = (uint8_t)(color & 0xFFU);
        }
      }
    }
  }

  if ((x >= active_lcd.width) || (y >= active_lcd.height) ||
      ((uint32_t)x + LCD_CHAR_ADVANCE > active_lcd.width) ||
      ((uint32_t)y + (LCD_CHAR_H * LCD_CHAR_SCALE) > active_lcd.height))
  {
    return;
  }

  if (!LCD_SetAddressWindow(x,
                            y,
                            (uint16_t)(x + LCD_CHAR_ADVANCE - 1U),
                            (uint16_t)(y + (LCD_CHAR_H * LCD_CHAR_SCALE) - 1U)))
  {
    return;
  }
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
  LCD_Select();
  (void)HAL_SPI_Transmit(&hspi2, pixels, (uint16_t)sizeof(pixels), LCD_SPI_TIMEOUT_MS);
  LCD_Unselect();
}

void LCD_DrawString(uint16_t x, uint16_t y, const char *text, uint16_t fg, uint16_t bg)
{
  while ((text != NULL) && (*text != '\0') &&
         ((uint32_t)x + LCD_CHAR_ADVANCE <= active_lcd.width))
  {
    LCD_DrawChar(x, y, *text, fg, bg);
    x = (uint16_t)(x + LCD_CHAR_ADVANCE);
    text++;
  }
}
