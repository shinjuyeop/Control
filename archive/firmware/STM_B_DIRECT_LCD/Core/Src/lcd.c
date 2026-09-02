#include "lcd.h"

#include <stddef.h>

#include "cmsis_os.h"
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
#define LCD_CMD_RDDPM   0x0AU

#define LCD_SPI_TIMEOUT_MS 100U
#define LCD_CHAR_W 5U
#define LCD_CHAR_H 7U
#define LCD_DEFAULT_CHAR_SCALE 3U
#define LCD_MAX_CHAR_SCALE 6U
#define LCD_PIXEL_BUFFER_BYTES \
  (LCD_LANDSCAPE_WIDTH * LCD_COMPOSE_BAND_MAX_ROWS * 2U)
#define LCD_INIT_MAX_DATA 14U

#define LCD_POWER_MODE_SLEEP_OUT  (1U << 4)
#define LCD_POWER_MODE_DISPLAY_ON (1U << 2)
#define LCD_POWER_MODE_REQUIRED \
  (LCD_POWER_MODE_SLEEP_OUT | LCD_POWER_MODE_DISPLAY_ON)

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
static uint8_t pixel_buffer[LCD_PIXEL_BUFFER_BYTES];
static volatile uint8_t dma_complete;
static volatile uint8_t dma_error;
static uint8_t compose_band_active;
static uint16_t compose_band_y;
static uint16_t compose_band_height;
volatile uint32_t LCD_ERROR_COUNT = 0U;
volatile uint32_t LCD_LAST_ERROR = LCD_ERROR_NONE;

static bool LCD_Fail(LCD_Error_t error)
{
  LCD_LAST_ERROR = (uint32_t)error;
  LCD_ERROR_COUNT++;
  return false;
}

/* MSP4030 vendor sequence for its TN panel and ST7796S controller. */
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
    ['%'] = {0x63,0x13,0x08,0x64,0x63},
    ['+'] = {0x08,0x08,0x3E,0x08,0x08},
    ['-'] = {0x08,0x08,0x08,0x08,0x08},
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
    ['?'] = {0x02,0x01,0x51,0x09,0x06},
    ['A'] = {0x7E,0x11,0x11,0x11,0x7E},
    ['B'] = {0x7F,0x49,0x49,0x49,0x36},
    ['C'] = {0x3E,0x41,0x41,0x41,0x22},
    ['D'] = {0x7F,0x41,0x41,0x22,0x1C},
    ['E'] = {0x7F,0x49,0x49,0x49,0x41},
    ['F'] = {0x7F,0x09,0x09,0x09,0x01},
    ['G'] = {0x3E,0x41,0x49,0x49,0x7A},
    ['H'] = {0x7F,0x08,0x08,0x08,0x7F},
    ['I'] = {0x00,0x41,0x7F,0x41,0x00},
    ['J'] = {0x20,0x40,0x41,0x3F,0x01},
    ['K'] = {0x7F,0x08,0x14,0x22,0x41},
    ['L'] = {0x7F,0x40,0x40,0x40,0x40},
    ['M'] = {0x7F,0x02,0x0C,0x02,0x7F},
    ['N'] = {0x7F,0x04,0x08,0x10,0x7F},
    ['O'] = {0x3E,0x41,0x41,0x41,0x3E},
    ['P'] = {0x7F,0x09,0x09,0x09,0x06},
    ['Q'] = {0x3E,0x41,0x51,0x21,0x5E},
    ['R'] = {0x7F,0x09,0x19,0x29,0x46},
    ['S'] = {0x46,0x49,0x49,0x49,0x31},
    ['T'] = {0x01,0x01,0x7F,0x01,0x01},
    ['U'] = {0x3F,0x40,0x40,0x40,0x3F},
    ['V'] = {0x1F,0x20,0x40,0x20,0x1F},
    ['W'] = {0x7F,0x20,0x18,0x20,0x7F},
    ['X'] = {0x63,0x14,0x08,0x14,0x63},
    ['Y'] = {0x03,0x04,0x78,0x04,0x03},
    ['Z'] = {0x61,0x51,0x49,0x45,0x43},
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
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
  LCD_Select();
  HAL_StatusTypeDef status =
      HAL_SPI_Transmit(&hspi2, &command, 1U, LCD_SPI_TIMEOUT_MS);
  LCD_Unselect();
  return (status == HAL_OK) ? true : LCD_Fail(LCD_ERROR_COMMAND_TX);
}

static bool LCD_WriteData(const uint8_t *data, uint16_t length)
{
  if ((data == NULL) || (length == 0U))
  {
    return false;
  }

  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
  LCD_Select();
  HAL_StatusTypeDef status = HAL_SPI_Transmit(
      &hspi2, (uint8_t *)data, length, LCD_SPI_TIMEOUT_MS);
  LCD_Unselect();
  return (status == HAL_OK) ? true : LCD_Fail(LCD_ERROR_DATA_TX);
}

static bool LCD_WriteCommandData(uint8_t command,
                                 const uint8_t *data,
                                 uint16_t length)
{
  return LCD_WriteCommand(command) &&
         ((length == 0U) || LCD_WriteData(data, length));
}

static bool LCD_TransmitPixels(uint8_t *data, uint16_t length)
{
  const uint32_t start = HAL_GetTick();
  dma_complete = 0U;
  dma_error = 0U;

  if (HAL_SPI_Transmit_DMA(&hspi2, data, length) != HAL_OK)
  {
    (void)HAL_SPI_Abort(&hspi2);
    return LCD_Fail(LCD_ERROR_DMA_START);
  }

  while ((dma_complete == 0U) && (dma_error == 0U))
  {
    if ((uint32_t)(HAL_GetTick() - start) >= LCD_SPI_TIMEOUT_MS)
    {
      (void)HAL_SPI_Abort(&hspi2);
      return LCD_Fail(LCD_ERROR_DMA_TIMEOUT);
    }
    osDelay(1U);
  }
  if (dma_error != 0U)
  {
    (void)HAL_SPI_Abort(&hspi2);
    return LCD_Fail(LCD_ERROR_DMA_CALLBACK);
  }
  return true;
}

static bool LCD_SetAddressWindow(uint16_t x0,
                                 uint16_t y0,
                                 uint16_t x1,
                                 uint16_t y1)
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
    madctl = LCD_MADCTL_BGR | LCD_MADCTL_MV;
  }
  else
  {
    return false;
  }
  active_lcd = *lcd;
  compose_band_active = 0U;

  LCD_SetBacklight(false);
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(100U);
  HAL_GPIO_WritePin(LCD_RST_GPIO_Port, LCD_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(50U);

  for (uint32_t i = 0U;
       i < (sizeof(msp4030_init) / sizeof(msp4030_init[0]));
       ++i)
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

  if (!LCD_WriteCommand(LCD_CMD_NORON) ||
      !LCD_WriteCommand(LCD_CMD_SLPOUT))
  {
    return false;
  }
  HAL_Delay(120U);
  if (!LCD_WriteCommand(LCD_CMD_DISPON))
  {
    return false;
  }
  HAL_Delay(20U);
  return true;
}

void LCD_SetBacklight(bool enabled)
{
  HAL_GPIO_WritePin(LCD_BL_GPIO_Port,
                    LCD_BL_Pin,
                    enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool LCD_ReadPowerMode(uint8_t *power_mode)
{
  uint8_t command = LCD_CMD_RDDPM;
  uint8_t tx_data[2] = {0U, 0U};
  uint8_t rx_data[2] = {0U, 0U};

  if (power_mode == NULL)
  {
    return false;
  }

  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);
  LCD_Select();
  if (HAL_SPI_Transmit(&hspi2, &command, 1U, LCD_SPI_TIMEOUT_MS) != HAL_OK)
  {
    LCD_Unselect();
    return LCD_Fail(LCD_ERROR_READ_COMMAND);
  }
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
  if (HAL_SPI_TransmitReceive(&hspi2,
                              tx_data,
                              rx_data,
                              sizeof(tx_data),
                              LCD_SPI_TIMEOUT_MS) != HAL_OK)
  {
    LCD_Unselect();
    return LCD_Fail(LCD_ERROR_READ_DATA);
  }
  LCD_Unselect();
  *power_mode = rx_data[1];
  return true;
}

bool LCD_IsHealthy(void)
{
  uint8_t power_mode;
  if (!LCD_ReadPowerMode(&power_mode))
  {
    return false;
  }
  if ((power_mode & LCD_POWER_MODE_REQUIRED) != LCD_POWER_MODE_REQUIRED)
  {
    return LCD_Fail(LCD_ERROR_HEALTH_STATUS);
  }
  return true;
}

bool LCD_BeginComposeBand(uint16_t y, uint16_t height)
{
  if ((compose_band_active != 0U) || (height == 0U) ||
      (height > LCD_COMPOSE_BAND_MAX_ROWS) ||
      (y >= active_lcd.height) ||
      ((uint32_t)y + height > active_lcd.height))
  {
    return false;
  }

  const uint32_t bytes = (uint32_t)active_lcd.width * height * 2U;
  for (uint32_t i = 0U; i < bytes; ++i)
  {
    pixel_buffer[i] = 0U;
  }
  compose_band_y = y;
  compose_band_height = height;
  compose_band_active = 1U;
  return true;
}

bool LCD_EndComposeBand(void)
{
  if (compose_band_active == 0U)
  {
    return false;
  }

  const uint16_t y = compose_band_y;
  const uint16_t height = compose_band_height;
  compose_band_active = 0U;
  if (!LCD_SetAddressWindow(0U,
                            y,
                            (uint16_t)(active_lcd.width - 1U),
                            (uint16_t)(y + height - 1U)))
  {
    return false;
  }

  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
  LCD_Select();
  const bool success = LCD_TransmitPixels(
      pixel_buffer, (uint16_t)(active_lcd.width * height * 2U));
  LCD_Unselect();
  return success;
}

void LCD_CancelComposeBand(void)
{
  compose_band_active = 0U;
}

bool LCD_FillRect(uint16_t x,
                  uint16_t y,
                  uint16_t w,
                  uint16_t h,
                  uint16_t color)
{
  if ((x >= active_lcd.width) || (y >= active_lcd.height) ||
      (w == 0U) || (h == 0U))
  {
    return false;
  }
  if ((uint32_t)x + w > active_lcd.width)
  {
    w = (uint16_t)(active_lcd.width - x);
  }
  if ((uint32_t)y + h > active_lcd.height)
  {
    h = (uint16_t)(active_lcd.height - y);
  }

  if (compose_band_active != 0U)
  {
    const uint16_t band_end =
        (uint16_t)(compose_band_y + compose_band_height);
    const uint16_t rect_end = (uint16_t)(y + h);
    const uint16_t clipped_y = (y > compose_band_y) ? y : compose_band_y;
    const uint16_t clipped_end =
        (rect_end < band_end) ? rect_end : band_end;
    if (clipped_y >= clipped_end)
    {
      return true;
    }

    for (uint16_t py = clipped_y; py < clipped_end; ++py)
    {
      uint32_t index =
          (((uint32_t)(py - compose_band_y) * active_lcd.width) + x) * 2U;
      for (uint16_t px = 0U; px < w; ++px)
      {
        pixel_buffer[index++] = (uint8_t)(color >> 8);
        pixel_buffer[index++] = (uint8_t)(color & 0xFFU);
      }
    }
    return true;
  }

  for (uint32_t i = 0U; i < sizeof(pixel_buffer); i += 2U)
  {
    pixel_buffer[i] = (uint8_t)(color >> 8);
    pixel_buffer[i + 1U] = (uint8_t)(color & 0xFFU);
  }

  if (!LCD_SetAddressWindow(
          x, y, (uint16_t)(x + w - 1U), (uint16_t)(y + h - 1U)))
  {
    return false;
  }

  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
  LCD_Select();
  uint32_t remaining = (uint32_t)w * h * 2U;
  while (remaining > 0U)
  {
    const uint16_t chunk =
        (remaining > sizeof(pixel_buffer)) ? (uint16_t)sizeof(pixel_buffer)
                                           : (uint16_t)remaining;
    if (!LCD_TransmitPixels(pixel_buffer, chunk))
    {
      LCD_Unselect();
      return false;
    }
    remaining -= chunk;
  }
  LCD_Unselect();
  return true;
}

bool LCD_FillScreen(uint16_t color)
{
  return LCD_FillRect(0U, 0U, active_lcd.width, active_lcd.height, color);
}

bool LCD_DrawRect(uint16_t x,
                  uint16_t y,
                  uint16_t w,
                  uint16_t h,
                  uint16_t thickness,
                  uint16_t color)
{
  if ((thickness == 0U) || (w < (2U * thickness)) ||
      (h < (2U * thickness)))
  {
    return false;
  }
  return LCD_FillRect(x, y, w, thickness, color) &&
         LCD_FillRect(x, (uint16_t)(y + h - thickness), w, thickness, color) &&
         LCD_FillRect(x, y, thickness, h, color) &&
         LCD_FillRect((uint16_t)(x + w - thickness), y, thickness, h, color);
}

bool LCD_DrawCharScaled(uint16_t x,
                        uint16_t y,
                        char c,
                        uint8_t scale,
                        uint16_t fg,
                        uint16_t bg)
{
  if ((scale == 0U) || (scale > LCD_MAX_CHAR_SCALE))
  {
    return false;
  }
  const uint16_t width = (uint16_t)((LCD_CHAR_W + 1U) * scale);
  const uint16_t height = (uint16_t)(LCD_CHAR_H * scale);
  if ((x >= active_lcd.width) || (y >= active_lcd.height) ||
      ((uint32_t)x + width > active_lcd.width) ||
      ((uint32_t)y + height > active_lcd.height))
  {
    return false;
  }
  if ((uint8_t)c >= (sizeof(font5x7) / sizeof(font5x7[0])))
  {
    c = '?';
  }

  const uint8_t *glyph = font5x7[(uint8_t)c];
  if (compose_band_active != 0U)
  {
    const uint16_t band_end =
        (uint16_t)(compose_band_y + compose_band_height);
    for (uint16_t output_y = 0U; output_y < height; ++output_y)
    {
      const uint16_t py = (uint16_t)(y + output_y);
      if ((py < compose_band_y) || (py >= band_end))
      {
        continue;
      }
      const uint8_t glyph_row = (uint8_t)(output_y / scale);
      for (uint16_t output_x = 0U; output_x < width; ++output_x)
      {
        const uint8_t glyph_column = (uint8_t)(output_x / scale);
        uint16_t color = bg;
        if ((glyph_column < LCD_CHAR_W) &&
            ((glyph[glyph_column] & (1U << glyph_row)) != 0U))
        {
          color = fg;
        }
        const uint16_t px = (uint16_t)(x + output_x);
        const uint32_t index =
            (((uint32_t)(py - compose_band_y) * active_lcd.width) + px) * 2U;
        pixel_buffer[index] = (uint8_t)(color >> 8);
        pixel_buffer[index + 1U] = (uint8_t)(color & 0xFFU);
      }
    }
    return true;
  }

  uint32_t index = 0U;
  for (uint8_t row = 0U; row < LCD_CHAR_H; ++row)
  {
    for (uint8_t row_scale = 0U; row_scale < scale; ++row_scale)
    {
      for (uint8_t column = 0U; column < (LCD_CHAR_W + 1U); ++column)
      {
        uint16_t color = bg;
        if ((column < LCD_CHAR_W) &&
            ((glyph[column] & (1U << row)) != 0U))
        {
          color = fg;
        }
        for (uint8_t column_scale = 0U;
             column_scale < scale;
             ++column_scale)
        {
          pixel_buffer[index++] = (uint8_t)(color >> 8);
          pixel_buffer[index++] = (uint8_t)(color & 0xFFU);
        }
      }
    }
  }

  if (!LCD_SetAddressWindow(x,
                            y,
                            (uint16_t)(x + width - 1U),
                            (uint16_t)(y + height - 1U)))
  {
    return false;
  }
  HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);
  LCD_Select();
  const bool success = LCD_TransmitPixels(pixel_buffer, (uint16_t)index);
  LCD_Unselect();
  return success;
}

bool LCD_DrawStringScaled(uint16_t x,
                          uint16_t y,
                          const char *text,
                          uint8_t scale,
                          uint16_t fg,
                          uint16_t bg)
{
  if ((text == NULL) || (scale == 0U) || (scale > LCD_MAX_CHAR_SCALE))
  {
    return false;
  }
  const uint16_t advance = (uint16_t)((LCD_CHAR_W + 1U) * scale);
  while (*text != '\0')
  {
    if ((uint32_t)x + advance > active_lcd.width)
    {
      return false;
    }
    if (!LCD_DrawCharScaled(x, y, *text, scale, fg, bg))
    {
      return false;
    }
    x = (uint16_t)(x + advance);
    ++text;
  }
  return true;
}

bool LCD_DrawChar(uint16_t x,
                  uint16_t y,
                  char c,
                  uint16_t fg,
                  uint16_t bg)
{
  return LCD_DrawCharScaled(
      x, y, c, LCD_DEFAULT_CHAR_SCALE, fg, bg);
}

bool LCD_DrawString(uint16_t x,
                    uint16_t y,
                    const char *text,
                    uint16_t fg,
                    uint16_t bg)
{
  return LCD_DrawStringScaled(
      x, y, text, LCD_DEFAULT_CHAR_SCALE, fg, bg);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &hspi2)
  {
    dma_complete = 1U;
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == &hspi2)
  {
    dma_error = 1U;
  }
}
