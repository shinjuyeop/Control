#include "include/st7796_lcd.h"

#include <SPI.h>

#include "include/dashboard_timing.h"

namespace
{

static constexpr uint8_t LCD_CS_PIN = D10;
static constexpr uint8_t LCD_DC_PIN = D9;
static constexpr uint8_t LCD_RST_PIN = D8;
static constexpr uint8_t LCD_BL_PIN = D7;

static constexpr uint32_t LCD_SPI_HZ = 12000000U;
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

struct InitCommand
{
  uint8_t command;
  uint8_t length;
  uint8_t data[LCD_INIT_MAX_DATA];
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

static uint8_t pixel_stream_buffer[PIXEL_CHUNK_BYTES];
#if defined(DASHBOARD_BENCH_INPUT) && \
    defined(DASHBOARD_TIMING_TRANSPORT_PROFILE)
static uint16_t transport_calibration_pixels[PIXEL_CHUNK_BYTES / 2U];
#endif

static const SPISettings lcd_spi_settings(
    LCD_SPI_HZ, MSBFIRST, SPI_MODE3);

void transferByte(uint8_t value)
{
  const uint32_t started_us = DashboardTiming::beginMeasure();
  SPI.transfer(value);
  DashboardTiming::transportSpiByte(started_us);
}

void transferBuffer(uint8_t *data, size_t length)
{
  const uint32_t started_us = DashboardTiming::beginMeasure();
  SPI.transfer(data, length);
  DashboardTiming::transportSpiBuffer(started_us, (uint16_t)length);
}

void writeCommand(uint8_t command)
{
  uint32_t started_us = DashboardTiming::beginMeasure();
  SPI.beginTransaction(lcd_spi_settings);
  digitalWrite(LCD_DC_PIN, LOW);
  digitalWrite(LCD_CS_PIN, LOW);
  DashboardTiming::transportTransactionBegin(started_us);
  transferByte(command);
  started_us = DashboardTiming::beginMeasure();
  digitalWrite(LCD_CS_PIN, HIGH);
  SPI.endTransaction();
  DashboardTiming::transportTransactionEnd(started_us);
}

void writeData(const uint8_t *data, size_t length)
{
  uint32_t started_us = DashboardTiming::beginMeasure();
  SPI.beginTransaction(lcd_spi_settings);
  digitalWrite(LCD_DC_PIN, HIGH);
  digitalWrite(LCD_CS_PIN, LOW);
  DashboardTiming::transportTransactionBegin(started_us);
  for (size_t i = 0U; i < length; ++i)
  {
    transferByte(data[i]);
  }
  started_us = DashboardTiming::beginMeasure();
  digitalWrite(LCD_CS_PIN, HIGH);
  SPI.endTransaction();
  DashboardTiming::transportTransactionEnd(started_us);
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

  uint32_t started_us = DashboardTiming::beginMeasure();
  SPI.beginTransaction(lcd_spi_settings);
  digitalWrite(LCD_DC_PIN, HIGH);
  digitalWrite(LCD_CS_PIN, LOW);
  DashboardTiming::transportTransactionBegin(started_us);
  while (remaining != 0U)
  {
    const size_t chunk =
        (remaining > sizeof(pixels)) ? sizeof(pixels) : (size_t)remaining;
    started_us = DashboardTiming::beginMeasure();
    for (size_t i = 0U; i < chunk; i += 2U)
    {
      pixels[i] = high;
      pixels[i + 1U] = low;
    }
    DashboardTiming::transportSolidPack(started_us, (uint16_t)(chunk / 2U));
    transferBuffer(pixels, chunk);
    remaining -= chunk;
  }
  started_us = DashboardTiming::beginMeasure();
  digitalWrite(LCD_CS_PIN, HIGH);
  SPI.endTransaction();
  DashboardTiming::transportTransactionEnd(started_us);
}

void fillScreen(uint16_t color)
{
  fillRect(0U, 0U, WIDTH, HEIGHT, color);
}

void beginPixelStream()
{
  beginPixelStream(0U, 0U, WIDTH - 1U, HEIGHT - 1U);
}

void beginPixelStream(uint16_t x0,
                      uint16_t y0,
                      uint16_t x1,
                      uint16_t y1)
{
  setAddressWindow(x0, y0, x1, y1);
  const uint32_t started_us = DashboardTiming::beginMeasure();
  SPI.beginTransaction(lcd_spi_settings);
  digitalWrite(LCD_DC_PIN, HIGH);
  digitalWrite(LCD_CS_PIN, LOW);
  DashboardTiming::transportTransactionBegin(started_us);
}

void writePixelRow(const uint16_t *pixels, size_t count)
{
  while (count != 0U)
  {
    const size_t pixel_count =
        (count > (PIXEL_CHUNK_BYTES / 2U))
            ? (PIXEL_CHUNK_BYTES / 2U) : count;
    const uint32_t started_us = DashboardTiming::beginMeasure();
    for (size_t index = 0U; index < pixel_count; ++index)
    {
      pixel_stream_buffer[index * 2U] = (uint8_t)(pixels[index] >> 8);
      pixel_stream_buffer[index * 2U + 1U] =
          (uint8_t)(pixels[index] & 0xFFU);
    }
    DashboardTiming::transportRgb565Pack(started_us, (uint16_t)pixel_count);
    transferBuffer(pixel_stream_buffer, pixel_count * 2U);
    pixels += pixel_count;
    count -= pixel_count;
  }
}

void endPixelStream()
{
  const uint32_t started_us = DashboardTiming::beginMeasure();
  digitalWrite(LCD_CS_PIN, HIGH);
  SPI.endTransaction();
  DashboardTiming::transportTransactionEnd(started_us);
}

#if defined(DASHBOARD_BENCH_INPUT) && \
    defined(DASHBOARD_TIMING_TRANSPORT_PROFILE)
void runTransportCalibration()
{
  for (uint16_t index = 0U;
       index < PIXEL_CHUNK_BYTES / 2U;
       ++index)
  {
    transport_calibration_pixels[index] = (uint16_t)(index * 257U);
  }

  for (uint8_t repetition = 0U; repetition < 8U; ++repetition)
  {
    beginPixelStream(0U, 0U, WIDTH - 1U, HEIGHT - 1U);
    for (uint16_t bytes = 2U; bytes <= PIXEL_CHUNK_BYTES; ++bytes)
      transferBuffer(pixel_stream_buffer, bytes);
    endPixelStream();

    for (uint16_t pixel_count = 1U;
         pixel_count <= PIXEL_CHUNK_BYTES / 2U;
         ++pixel_count)
    {
      beginPixelStream(0U, 0U, (uint16_t)(pixel_count - 1U), 0U);
      writePixelRow(transport_calibration_pixels, pixel_count);
      endPixelStream();
      fillRect(0U, 1U, pixel_count, 1U, (uint16_t)(0x1234U + pixel_count));
    }
  }
}
#endif

}  // namespace St7796Lcd
