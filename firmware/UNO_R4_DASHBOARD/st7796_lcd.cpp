#include "st7796_lcd.h"

#include <SPI.h>

namespace
{

static constexpr uint8_t LCD_CS_PIN = D10;
static constexpr uint8_t LCD_DC_PIN = D9;
static constexpr uint8_t LCD_RST_PIN = D8;
static constexpr uint8_t LCD_BL_PIN = D7;

static constexpr uint32_t LCD_SPI_HZ = 1000000U;
static constexpr uint8_t LCD_CMD_SLPOUT = 0x11U;
static constexpr uint8_t LCD_CMD_NORON = 0x13U;
static constexpr uint8_t LCD_CMD_DISPON = 0x29U;
static constexpr uint8_t LCD_CMD_CASET = 0x2AU;
static constexpr uint8_t LCD_CMD_RASET = 0x2BU;
static constexpr uint8_t LCD_CMD_RAMWR = 0x2CU;
static constexpr uint8_t LCD_CMD_MADCTL = 0x36U;
static constexpr uint8_t LCD_CMD_COLMOD = 0x3AU;

static constexpr uint8_t LCD_INIT_MAX_DATA = 14U;
static constexpr size_t PIXEL_CHUNK_BYTES = 256U;
static constexpr uint8_t FONT_WIDTH = 5U;
static constexpr uint8_t FONT_HEIGHT = 7U;
static constexpr uint8_t MAX_FONT_SCALE = 6U;
static constexpr size_t GLYPH_BUFFER_BYTES =
    (FONT_WIDTH + 1U) * MAX_FONT_SCALE * FONT_HEIGHT * MAX_FONT_SCALE * 2U;

struct InitCommand
{
  uint8_t command;
  uint8_t length;
  uint8_t data[LCD_INIT_MAX_DATA];
};

struct FontGlyph
{
  char character;
  uint8_t columns[FONT_WIDTH];
};

/* Effective STM_B MSP4030 sequence. MADCTL 0x28 is BGR + landscape MV. */
static const InitCommand msp4030_init[] = {
    {0xF0U, 1U, {0xC3U}},
    {0xF0U, 1U, {0x96U}},
    {LCD_CMD_MADCTL, 1U, {0x28U}},
    {LCD_CMD_COLMOD, 1U, {0x05U}},
    {0xB0U, 1U, {0x80U}},
    {0xB6U, 2U, {0x00U, 0x02U}},
    {0xB5U, 4U, {0x02U, 0x03U, 0x00U, 0x04U}},
    {0xB1U, 2U, {0x80U, 0x10U}},
    {0xB4U, 1U, {0x00U}},
    {0xB7U, 1U, {0xC6U}},
    {0xC5U, 1U, {0x1CU}},
    {0xE4U, 1U, {0x31U}},
    {0xE8U, 8U, {0x40U, 0x8AU, 0x00U, 0x00U,
                 0x29U, 0x19U, 0xA5U, 0x33U}},
    {0xC2U, 1U, {0xA7U}},
    {0xE0U, 14U, {0xF0U, 0x09U, 0x13U, 0x12U, 0x12U, 0x2BU, 0x3CU,
                  0x44U, 0x4BU, 0x1BU, 0x18U, 0x17U, 0x1DU, 0x21U}},
    {0xE1U, 14U, {0xF0U, 0x09U, 0x13U, 0x0CU, 0x0DU, 0x27U, 0x3BU,
                  0x44U, 0x4DU, 0x0BU, 0x17U, 0x17U, 0x1DU, 0x21U}},
    {0xF0U, 1U, {0x3CU}},
    {0xF0U, 1U, {0x69U}},
};

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

static uint8_t glyph_buffer[GLYPH_BUFFER_BYTES];

static const SPISettings lcd_spi_settings(
    LCD_SPI_HZ, MSBFIRST, SPI_MODE3);

void writeCommand(uint8_t command)
{
  SPI.beginTransaction(lcd_spi_settings);
  digitalWrite(LCD_DC_PIN, LOW);
  digitalWrite(LCD_CS_PIN, LOW);
  SPI.transfer(command);
  digitalWrite(LCD_CS_PIN, HIGH);
  SPI.endTransaction();
}

void writeData(const uint8_t *data, size_t length)
{
  SPI.beginTransaction(lcd_spi_settings);
  digitalWrite(LCD_DC_PIN, HIGH);
  digitalWrite(LCD_CS_PIN, LOW);
  for (size_t i = 0U; i < length; ++i)
  {
    SPI.transfer(data[i]);
  }
  digitalWrite(LCD_CS_PIN, HIGH);
  SPI.endTransaction();
}

void writeCommandData(uint8_t command, const uint8_t *data, size_t length)
{
  writeCommand(command);
  if (length != 0U) writeData(data, length);
}

void setAddressWindow(uint16_t x0,
                      uint16_t y0,
                      uint16_t x1,
                      uint16_t y1)
{
  uint8_t data[4] = {
      (uint8_t)(x0 >> 8),
      (uint8_t)(x0 & 0xFFU),
      (uint8_t)(x1 >> 8),
      (uint8_t)(x1 & 0xFFU)};
  writeCommandData(LCD_CMD_CASET, data, sizeof(data));

  data[0] = (uint8_t)(y0 >> 8);
  data[1] = (uint8_t)(y0 & 0xFFU);
  data[2] = (uint8_t)(y1 >> 8);
  data[3] = (uint8_t)(y1 & 0xFFU);
  writeCommandData(LCD_CMD_RASET, data, sizeof(data));
  writeCommand(LCD_CMD_RAMWR);
}

const uint8_t *lookupGlyph(char character)
{
  for (const FontGlyph &glyph : font5x7)
  {
    if (glyph.character == character) return glyph.columns;
  }
  for (const FontGlyph &glyph : font5x7)
  {
    if (glyph.character == '?') return glyph.columns;
  }
  return font5x7[0].columns;
}

}  // namespace

namespace St7796Lcd
{

void setBacklight(bool enabled)
{
  digitalWrite(LCD_BL_PIN, enabled ? HIGH : LOW);
}

void begin()
{
  pinMode(LCD_CS_PIN, OUTPUT);
  pinMode(LCD_DC_PIN, OUTPUT);
  pinMode(LCD_RST_PIN, OUTPUT);
  pinMode(LCD_BL_PIN, OUTPUT);

  digitalWrite(LCD_CS_PIN, HIGH);
  digitalWrite(LCD_DC_PIN, HIGH);
  digitalWrite(LCD_RST_PIN, HIGH);
  setBacklight(false);
  SPI.begin();

  digitalWrite(LCD_RST_PIN, LOW);
  delay(100U);
  digitalWrite(LCD_RST_PIN, HIGH);
  delay(50U);

  for (const InitCommand &entry : msp4030_init)
  {
    writeCommandData(entry.command, entry.data, entry.length);
  }

  writeCommand(LCD_CMD_NORON);
  writeCommand(LCD_CMD_SLPOUT);
  delay(120U);
  writeCommand(LCD_CMD_DISPON);
  delay(20U);
}

void fillRect(uint16_t x,
              uint16_t y,
              uint16_t width,
              uint16_t height,
              uint16_t color)
{
  if ((x >= WIDTH) || (y >= HEIGHT) || (width == 0U) || (height == 0U))
  {
    return;
  }
  if ((uint32_t)x + width > WIDTH) width = (uint16_t)(WIDTH - x);
  if ((uint32_t)y + height > HEIGHT) height = (uint16_t)(HEIGHT - y);

  setAddressWindow(
      x, y, (uint16_t)(x + width - 1U), (uint16_t)(y + height - 1U));

  uint8_t pixels[PIXEL_CHUNK_BYTES];
  const uint8_t high = (uint8_t)(color >> 8);
  const uint8_t low = (uint8_t)(color & 0xFFU);
  uint32_t remaining = (uint32_t)width * height * 2U;

  SPI.beginTransaction(lcd_spi_settings);
  digitalWrite(LCD_DC_PIN, HIGH);
  digitalWrite(LCD_CS_PIN, LOW);
  while (remaining != 0U)
  {
    const size_t chunk =
        (remaining > sizeof(pixels)) ? sizeof(pixels) : (size_t)remaining;
    for (size_t i = 0U; i < chunk; i += 2U)
    {
      pixels[i] = high;
      pixels[i + 1U] = low;
    }
    SPI.transfer(pixels, chunk);
    remaining -= chunk;
  }
  digitalWrite(LCD_CS_PIN, HIGH);
  SPI.endTransaction();
}

void fillScreen(uint16_t color)
{
  fillRect(0U, 0U, WIDTH, HEIGHT, color);
}

void drawRect(uint16_t x,
              uint16_t y,
              uint16_t width,
              uint16_t height,
              uint16_t thickness,
              uint16_t color)
{
  if ((thickness == 0U) || (width < (2U * thickness)) ||
      (height < (2U * thickness)))
  {
    return;
  }
  fillRect(x, y, width, thickness, color);
  fillRect(x, (uint16_t)(y + height - thickness), width, thickness, color);
  fillRect(x, y, thickness, height, color);
  fillRect((uint16_t)(x + width - thickness), y, thickness, height, color);
}

void drawCharScaled(uint16_t x,
                    uint16_t y,
                    char character,
                    uint8_t scale,
                    uint16_t foreground,
                    uint16_t background)
{
  if ((scale == 0U) || (scale > MAX_FONT_SCALE)) return;
  const uint16_t width = (uint16_t)((FONT_WIDTH + 1U) * scale);
  const uint16_t height = (uint16_t)(FONT_HEIGHT * scale);
  if (((uint32_t)x + width > WIDTH) || ((uint32_t)y + height > HEIGHT)) return;

  const uint8_t *glyph = lookupGlyph(character);
  size_t index = 0U;
  for (uint8_t row = 0U; row < FONT_HEIGHT; ++row)
  {
    for (uint8_t row_scale = 0U; row_scale < scale; ++row_scale)
    {
      for (uint8_t column = 0U; column < (FONT_WIDTH + 1U); ++column)
      {
        uint16_t color = background;
        if ((column < FONT_WIDTH) &&
            ((glyph[column] & (1U << row)) != 0U))
        {
          color = foreground;
        }
        for (uint8_t column_scale = 0U;
             column_scale < scale;
             ++column_scale)
        {
          glyph_buffer[index++] = (uint8_t)(color >> 8);
          glyph_buffer[index++] = (uint8_t)(color & 0xFFU);
        }
      }
    }
  }

  setAddressWindow(
      x, y, (uint16_t)(x + width - 1U), (uint16_t)(y + height - 1U));
  SPI.beginTransaction(lcd_spi_settings);
  digitalWrite(LCD_DC_PIN, HIGH);
  digitalWrite(LCD_CS_PIN, LOW);
  SPI.transfer(glyph_buffer, index);
  digitalWrite(LCD_CS_PIN, HIGH);
  SPI.endTransaction();
}

void drawStringScaled(uint16_t x,
                      uint16_t y,
                      const char *text,
                      uint8_t scale,
                      uint16_t foreground,
                      uint16_t background)
{
  if ((text == nullptr) || (scale == 0U) || (scale > MAX_FONT_SCALE)) return;
  const uint16_t advance = (uint16_t)((FONT_WIDTH + 1U) * scale);
  while (*text != '\0')
  {
    if ((uint32_t)x + advance > WIDTH) return;
    drawCharScaled(x, y, *text, scale, foreground, background);
    x = (uint16_t)(x + advance);
    ++text;
  }
}

}  // namespace St7796Lcd
