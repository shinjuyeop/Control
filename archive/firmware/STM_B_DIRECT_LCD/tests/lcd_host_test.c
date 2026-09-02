#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "lcd.h"
#include "main.h"

#define MAX_EVENTS 10000U
#define MAX_EVENT_DATA 16U

typedef struct
{
  GPIO_PinState dc;
  uint16_t length;
  uint8_t data[MAX_EVENT_DATA];
} SpiEvent_t;

SPI_HandleTypeDef hspi2;

static SpiEvent_t events[MAX_EVENTS];
static size_t event_count;
static uint32_t transmitted_bytes;
static GPIO_PinState dc_state;
static GPIO_PinState bl_state;
static uint32_t delays[8];
static size_t delay_count;
static bool fail_spi;
static bool fail_dma;
static bool suppress_dma_complete;
static uint8_t power_mode;
static uint32_t tick_ms;
static uint32_t abort_count;

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *spi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *spi);

void HAL_GPIO_WritePin(void *port, uint16_t pin, GPIO_PinState state)
{
  (void)port;
  if (pin == LCD_DC_Pin)
  {
    dc_state = state;
  }
  else if (pin == LCD_BL_Pin)
  {
    bl_state = state;
  }
}

void HAL_Delay(uint32_t delay_ms)
{
  assert(delay_count < (sizeof(delays) / sizeof(delays[0])));
  delays[delay_count++] = delay_ms;
}

void osDelay(uint32_t delay_ms)
{
  tick_ms += delay_ms;
}

uint32_t HAL_GetTick(void)
{
  return tick_ms;
}

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *spi,
                                   uint8_t *data,
                                   uint16_t length,
                                   uint32_t timeout_ms)
{
  assert(spi == &hspi2);
  assert(timeout_ms == 100U);
  if (fail_spi)
  {
    return HAL_ERROR;
  }

  assert(event_count < MAX_EVENTS);
  events[event_count].dc = dc_state;
  events[event_count].length = length;
  memcpy(events[event_count].data,
         data,
         (length < MAX_EVENT_DATA) ? length : MAX_EVENT_DATA);
  event_count++;
  transmitted_bytes += length;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *spi,
                                       uint8_t *data,
                                       uint16_t length)
{
  if (fail_dma)
  {
    return HAL_ERROR;
  }
  HAL_StatusTypeDef status = HAL_SPI_Transmit(spi, data, length, 100U);
  if ((status == HAL_OK) && !suppress_dma_complete)
  {
    HAL_SPI_TxCpltCallback(spi);
  }
  return status;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi,
                                          uint8_t *tx_data,
                                          uint8_t *rx_data,
                                          uint16_t length,
                                          uint32_t timeout_ms)
{
  HAL_StatusTypeDef status =
      HAL_SPI_Transmit(spi, tx_data, length, timeout_ms);
  if ((status == HAL_OK) && (length >= 2U))
  {
    rx_data[0] = 0U;
    rx_data[1] = power_mode;
  }
  return status;
}

HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *spi)
{
  assert(spi == &hspi2);
  abort_count++;
  return HAL_OK;
}

static void ResetCapture(void)
{
  event_count = 0U;
  transmitted_bytes = 0U;
  dc_state = GPIO_PIN_RESET;
  bl_state = GPIO_PIN_RESET;
  delay_count = 0U;
  fail_spi = false;
  fail_dma = false;
  suppress_dma_complete = false;
  power_mode = 0x9CU;
  tick_ms = 0U;
  abort_count = 0U;
  memset(events, 0, sizeof(events));
  memset(delays, 0, sizeof(delays));
}

static void TestLandscapeInitialization(void)
{
  static const uint8_t expected_commands[] = {
      0xF0U, 0xF0U, 0x36U, 0x3AU, 0xB0U, 0xB6U, 0xB5U,
      0xB1U, 0xB4U, 0xB7U, 0xC5U, 0xE4U, 0xE8U, 0xC2U,
      0xE0U, 0xE1U, 0xF0U, 0xF0U, 0x13U, 0x11U, 0x29U};
  LCD_Handle_t lcd = {
      LCD_LANDSCAPE_WIDTH,
      LCD_LANDSCAPE_HEIGHT,
      LCD_ROTATION_LANDSCAPE};
  size_t command_index = 0U;

  ResetCapture();
  assert(LCD_Init(&lcd));
  assert(bl_state == GPIO_PIN_RESET);
  LCD_SetBacklight(true);
  assert(bl_state == GPIO_PIN_SET);
  assert(delay_count == 4U);
  assert(delays[0] == 100U);
  assert(delays[1] == 50U);
  assert(delays[2] == 120U);
  assert(delays[3] == 20U);

  for (size_t i = 0U; i < event_count; i++)
  {
    if (events[i].dc == GPIO_PIN_RESET)
    {
      assert(events[i].length == 1U);
      assert(command_index < sizeof(expected_commands));
      assert(events[i].data[0] == expected_commands[command_index++]);
    }
  }
  assert(command_index == sizeof(expected_commands));

  assert(events[4].dc == GPIO_PIN_RESET);
  assert(events[4].data[0] == 0x36U);
  assert(events[5].dc == GPIO_PIN_SET);
  assert(events[5].data[0] == 0x28U);
  assert(events[7].data[0] == 0x05U);
  assert(events[27].data[0] == 0xA7U);
}

static void TestLandscapeAddressing(void)
{
  ResetCapture();
  assert(LCD_FillRect(479U, 319U, 10U, 10U, LCD_COLOR_RED));

  assert(event_count == 6U);
  assert(transmitted_bytes == 13U);
  assert(events[0].data[0] == 0x2AU);
  assert(memcmp(events[1].data,
                (uint8_t[]){0x01U, 0xDFU, 0x01U, 0xDFU},
                4U) == 0);
  assert(events[2].data[0] == 0x2BU);
  assert(memcmp(events[3].data,
                (uint8_t[]){0x01U, 0x3FU, 0x01U, 0x3FU},
                4U) == 0);
  assert(events[4].data[0] == 0x2CU);
  assert(events[5].data[0] == 0xF8U);
  assert(events[5].data[1] == 0x00U);

  ResetCapture();
  assert(LCD_FillScreen(LCD_COLOR_BLACK));
  assert(event_count == 85U);
  assert(transmitted_bytes == (480U * 320U * 2U) + 11U);

  ResetCapture();
  assert(!LCD_DrawChar(470U, 310U, 'A', LCD_COLOR_WHITE, LCD_COLOR_BLACK));
  assert(event_count == 0U);
}

static void TestPowerModeAndDmaFailure(void)
{
  uint8_t mode = 0U;

  ResetCapture();
  assert(LCD_ReadPowerMode(&mode));
  assert(mode == 0x9CU);
  assert(LCD_IsHealthy());

  power_mode = 0x08U;
  assert(!LCD_IsHealthy());

  ResetCapture();
  fail_dma = true;
  assert(!LCD_FillRect(0U, 0U, 2U, 2U, LCD_COLOR_RED));
  assert(abort_count == 1U);

  ResetCapture();
  suppress_dma_complete = true;
  assert(!LCD_FillRect(0U, 0U, 2U, 2U, LCD_COLOR_RED));
  assert(abort_count == 1U);
  assert(tick_ms >= 100U);
}

static void TestComposedBand(void)
{
  ResetCapture();
  assert(LCD_BeginComposeBand(10U, 2U));
  assert(LCD_FillScreen(LCD_COLOR_WHITE));
  assert(LCD_FillRect(0U, 10U, 2U, 1U, LCD_COLOR_RED));
  assert(event_count == 0U);
  assert(LCD_EndComposeBand());

  assert(event_count == 6U);
  assert(transmitted_bytes == (480U * 2U * 2U) + 11U);
  assert(memcmp(events[3].data,
                (uint8_t[]){0x00U, 0x0AU, 0x00U, 0x0BU},
                4U) == 0);
  assert(memcmp(events[5].data,
                (uint8_t[]){0xF8U, 0x00U, 0xF8U, 0x00U,
                            0xFFU, 0xFFU, 0xFFU, 0xFFU},
                8U) == 0);
}

static void TestGeometryValidation(void)
{
  LCD_Handle_t portrait = {
      LCD_PORTRAIT_WIDTH,
      LCD_PORTRAIT_HEIGHT,
      LCD_ROTATION_PORTRAIT};
  LCD_Handle_t invalid = {
      LCD_LANDSCAPE_WIDTH,
      LCD_LANDSCAPE_HEIGHT,
      LCD_ROTATION_PORTRAIT};

  ResetCapture();
  assert(!LCD_Init(&invalid));
  assert(event_count == 0U);

  ResetCapture();
  assert(LCD_Init(&portrait));
  assert(events[5].data[0] == 0x48U);

  ResetCapture();
  fail_spi = true;
  assert(!LCD_Init(&portrait));
  assert(bl_state == GPIO_PIN_RESET);
}

int main(void)
{
  TestLandscapeInitialization();
  TestLandscapeAddressing();
  TestComposedBand();
  TestPowerModeAndDmaFailure();
  TestGeometryValidation();
  puts("lcd_host_test: PASS");
  return 0;
}
