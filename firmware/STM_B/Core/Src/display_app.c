#include "display_app.h"

#include <stdint.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
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

void Dashboard_Task(void *argument)
{
  LCD_Handle_t lcd = {
      LCD_LANDSCAPE_WIDTH,
      LCD_LANDSCAPE_HEIGHT,
      LCD_ROTATION_LANDSCAPE};
  uint32_t last_tick = 0U;
  float previous_kmh = -1.0f;
  uint8_t previous_brake = UINT8_MAX;
  uint8_t previous_gear = UINT8_MAX;
  uint8_t previous_mode = UINT8_MAX;
  uint8_t previous_vehicle_valid = UINT8_MAX;

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
      float kmh;
      uint8_t brake;
      uint8_t gear;
      uint8_t mode;
      uint8_t vehicle_valid;

      /* UART parser가 함께 갱신하는 속도/기어/모드를 일관된 snapshot으로 읽는다. */
      taskENTER_CRITICAL();
      kmh = CURRENT_SPEED_FROM_A * 3.6f;
      gear = GEAR_FROM_A;
      mode = MODE_FROM_A;
      vehicle_valid = VEHICLE_FRAME_VALID_FROM_A;
      brake = BRAKE_REQUEST;
      taskEXIT_CRITICAL();

      last_tick = now;

      /* Redrawing every field can exceed the update period on a slow SPI
       * link, so only changed regions are cleared and transferred. */
      if (vehicle_valid != previous_vehicle_valid)
      {
        LCD_FillRect(20U, 40U, 260U, 64U, LCD_COLOR_BLACK);
        LCD_FillRect(300U, 40U, 160U, 64U, LCD_COLOR_BLACK);
        LCD_FillRect(300U, 180U, 160U, 64U, LCD_COLOR_BLACK);

        if (vehicle_valid == 0U)
        {
          LCD_DrawString(
              20U, 80U, "COMM FAULT", LCD_COLOR_RED, LCD_COLOR_BLACK);
        }
        else
        {
          LCD_DrawString(
              20U, 40U, "Speed:", LCD_COLOR_WHITE, LCD_COLOR_BLACK);
          LCD_DrawString(
              300U, 40U, "Gear:", LCD_COLOR_WHITE, LCD_COLOR_BLACK);
          LCD_DrawString(
              300U, 180U, "Mode:", LCD_COLOR_WHITE, LCD_COLOR_BLACK);
        }
        previous_kmh = -1.0f;
        previous_gear = UINT8_MAX;
        previous_mode = UINT8_MAX;
        previous_vehicle_valid = vehicle_valid;
      }

      if (vehicle_valid != 0U)
      {
        if ((kmh < (previous_kmh - 0.05f)) ||
            (kmh > (previous_kmh + 0.05f)))
        {
          LCD_FillRect(20U, 80U, 260U, 24U, LCD_COLOR_BLACK);
          Dashboard_FormatSpeed(buffer, sizeof(buffer), kmh);
          LCD_DrawString(
              20U, 80U, buffer, LCD_COLOR_WHITE, LCD_COLOR_BLACK);
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
      }

      if (brake != previous_brake)
      {
        LCD_FillRect(20U, 220U, 260U, 24U, LCD_COLOR_BLACK);
        LCD_DrawString(20U,
                       220U,
                       (brake != 0U) ? "BRAKE ENGAGED" : "NORMAL",
                       (brake != 0U) ? LCD_COLOR_RED : LCD_COLOR_GREEN,
                       LCD_COLOR_BLACK);
        previous_brake = brake;
      }
    }

    osDelay(10U);
  }
}
