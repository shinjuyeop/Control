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

static void ResetCapture(void)
{
  event_count = 0U;
  transmitted_bytes = 0U;
  dc_state = GPIO_PIN_RESET;
  bl_state = GPIO_PIN_RESET;
  delay_count = 0U;
  fail_spi = false;
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
  LCD_FillRect(479U, 319U, 10U, 10U, LCD_COLOR_RED);

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
  LCD_FillScreen(LCD_COLOR_BLACK);
  assert(transmitted_bytes == (480U * 320U * 2U) + 11U);

  ResetCapture();
  LCD_DrawChar(470U, 310U, 'A', LCD_COLOR_WHITE, LCD_COLOR_BLACK);
  assert(event_count == 0U);
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
  TestGeometryValidation();
  puts("lcd_host_test: PASS");
  return 0;
}
