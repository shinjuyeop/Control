#include "display_app.h"

#include <stdint.h>
#include <stdio.h>

#include "cmsis_os.h"
#include "globals.h"
#include "lcd.h"
#include "main.h"

static void Dashboard_FormatSpeed(char *buffer, size_t size, float kmh)
{
  int32_t tenths;

  if (kmh < 0.0f)
  {
    kmh = 0.0f;
  }

  tenths = (int32_t)((kmh * 10.0f) + 0.5f);
  (void)snprintf(buffer,
                 size,
                 "%ld.%ld km/h",
                 (long)(tenths / 10),
                 (long)(tenths % 10));
}

void Dashboard_DrawStaticLayout(void)
{
  LCD_FillScreen(LCD_COLOR_BLACK);

  LCD_DrawString(20U,  40U,  "Speed:",  LCD_COLOR_WHITE, LCD_COLOR_BLACK);
  LCD_DrawString(300U, 40U,  "Gear:",   LCD_COLOR_WHITE, LCD_COLOR_BLACK);
  LCD_DrawString(20U,  180U, "STATUS:", LCD_COLOR_WHITE, LCD_COLOR_BLACK);
  LCD_DrawString(300U, 180U, "Mode:",   LCD_COLOR_WHITE, LCD_COLOR_BLACK);
}

void StartDisplay(void *argument)
{
  LCD_Handle_t lcd = {
      LCD_LANDSCAPE_WIDTH,
      LCD_LANDSCAPE_HEIGHT,
      LCD_ROTATION_LANDSCAPE};
  uint32_t last_tick = 0U;
  float previous_kmh = -1.0f;
  uint8_t previous_estop = UINT8_MAX;
  uint8_t previous_gear = UINT8_MAX;
  uint8_t previous_mode = UINT8_MAX;

  (void)argument;

  while (!LCD_Init(&lcd))
  {
    osDelay(1000U);
  }
  Dashboard_DrawStaticLayout();

  for (;;)
  {
    uint32_t now = HAL_GetTick();

    if ((now - last_tick) >= DISPLAY_UPDATE_MS)
    {
      char buffer[20];
      float kmh = CURRENT_SPEED_FROM_A * 3.6f;
      uint8_t estop = ESTOP_STATUS;
      uint8_t gear = GEAR_FROM_A;
      uint8_t mode = MODE_FROM_A;

      last_tick = now;

      /* Redrawing every field can exceed the update period on a slow SPI
       * link, so only changed regions are cleared and transferred. */
      if ((kmh < (previous_kmh - 0.05f)) || (kmh > (previous_kmh + 0.05f)))
      {
        LCD_FillRect(20U, 80U, 260U, 24U, LCD_COLOR_BLACK);
        Dashboard_FormatSpeed(buffer, sizeof(buffer), kmh);
        LCD_DrawString(20U, 80U, buffer, LCD_COLOR_WHITE, LCD_COLOR_BLACK);
        previous_kmh = kmh;
      }

      if (gear != previous_gear)
      {
        LCD_FillRect(300U, 80U, 140U, 24U, LCD_COLOR_BLACK);
        LCD_DrawString(300U,
                       80U,
                       (gear == 1U) ? "REV" : "FWD",
                       LCD_COLOR_WHITE,
                       LCD_COLOR_BLACK);
        previous_gear = gear;
      }

      if (mode != previous_mode)
      {
        LCD_FillRect(300U, 220U, 160U, 24U, LCD_COLOR_BLACK);
        LCD_DrawString(300U,
                       220U,
                       (mode == 1U) ? "AUTO" : "MANUAL",
                       LCD_COLOR_WHITE,
                       LCD_COLOR_BLACK);
        previous_mode = mode;
      }

      if (estop != previous_estop)
      {
        LCD_FillRect(20U, 220U, 240U, 24U, LCD_COLOR_BLACK);
        if (estop == 1U)
        {
          LCD_DrawString(20U, 220U, "!! ESTOP !!", LCD_COLOR_RED, LCD_COLOR_BLACK);
        }
        else
        {
          LCD_DrawString(20U, 220U, "NORMAL", LCD_COLOR_GREEN, LCD_COLOR_BLACK);
        }
        previous_estop = estop;
      }
    }

    osDelay(10U);
  }
}
