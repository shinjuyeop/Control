#include "display_app.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "board_uart_protocol.h"
#include "cmsis_os.h"
#include "globals.h"
#include "lcd.h"
#include "main.h"
#include "task.h"

#define UI_BG LCD_COLOR_WHITE
#define UI_TEXT LCD_COLOR_BLACK
#define UI_CARD LCD_COLOR_LIGHT_GRAY
#define UI_BORDER LCD_COLOR_DARK_GRAY
#define UI_AUTO LCD_COLOR_BLUE
#define UI_MANUAL LCD_COLOR_ORANGE
#define UI_OK LCD_COLOR_GREEN
#define UI_FAULT LCD_COLOR_RED

#define HV_VALUE_CHARS 14U
#define SPEED_CHARS 5U
#define METRIC_CHARS 8U
#define BOTTOM_LEFT_CHARS 20U
#define BOTTOM_RIGHT_CHARS 16U

volatile uint32_t DISPLAY_RECOVERY_COUNT = 0U;
volatile uint32_t DISPLAY_LAST_RECOVERY_ERROR = LCD_ERROR_NONE;

typedef struct
{
  float speed_mps;
  uint8_t gear;
  uint8_t mode;
  uint8_t vehicle_valid;
  BoardUartDriveDetail_t detail;
  uint8_t detail_valid;
  uint8_t estop;
  uint8_t brake;
} DashboardState_t;

typedef struct
{
  char text[9];
  uint16_t color;
  uint8_t length;
} BadgeView_t;

typedef struct
{
  char left[BOTTOM_LEFT_CHARS + 1U];
  char right[BOTTOM_RIGHT_CHARS + 1U];
  uint16_t color;
  uint16_t text_color;
} BottomView_t;

typedef struct
{
  uint32_t started_ms;
  uint8_t transition_count;
  uint8_t previous_gear;
  uint8_t initialized;
} GearResetGesture_t;

typedef struct
{
  uint32_t event_ms;
  int16_t previous_phase_da;
  int8_t phase_sign;
  uint8_t has_previous;
  uint8_t pending;
} PhaseDisturbanceMonitor_t;

static void Dashboard_ReadState(DashboardState_t *state)
{
  taskENTER_CRITICAL();
  state->speed_mps = CURRENT_SPEED_FROM_A;
  state->gear = GEAR_FROM_A;
  state->mode = MODE_FROM_A;
  state->vehicle_valid = VEHICLE_FRAME_VALID_FROM_A;
  state->detail = DRIVE_DETAIL_FROM_A;
  state->detail_valid = DRIVE_DETAIL_FRAME_VALID_FROM_A;
  state->estop = ESTOP_STATUS;
  state->brake = BRAKE_REQUEST;
  taskEXIT_CRITICAL();
}

static void FormatTenthsSigned(char *buffer,
                               size_t size,
                               int16_t value,
                               const char *unit)
{
  int32_t signed_value = value;
  int32_t magnitude = (signed_value < 0) ? -signed_value : signed_value;
  (void)snprintf(buffer,
                 size,
                 "%s%ld.%ld %s",
                 (signed_value < 0) ? "-" : "",
                 (long)(magnitude / 10),
                 (long)(magnitude % 10),
                 unit);
}

static void FormatTenthsUnsigned(char *buffer,
                                 size_t size,
                                 uint16_t value,
                                 const char *unit)
{
  (void)snprintf(buffer,
                 size,
                 "%lu.%lu %s",
                 (unsigned long)(value / 10U),
                 (unsigned long)(value % 10U),
                 unit);
}

static void FormatSpeed(char *buffer, size_t size, float speed_mps)
{
  float kmh = fabsf(speed_mps) * 3.6f;
  int32_t tenths = (int32_t)lroundf(kmh * 10.0f);
  (void)snprintf(buffer,
                 size,
                 "%ld.%ld",
                 (long)(tenths / 10),
                 (long)(tenths % 10));
}

static int32_t SpeedTenthsKmh(float speed_mps)
{
  return (int32_t)lroundf(fabsf(speed_mps) * 36.0f);
}

static void PadLeft(char *destination,
                    size_t width,
                    const char *source)
{
  size_t length = strlen(source);
  if (length > width) length = width;
  const size_t padding = width - length;
  memset(destination, ' ', padding);
  memcpy(&destination[padding], source, length);
  destination[width] = '\0';
}

static void PadRight(char *destination,
                     size_t width,
                     const char *source)
{
  size_t length = strlen(source);
  if (length > width) length = width;
  memcpy(destination, source, length);
  memset(&destination[length], ' ', width - length);
  destination[width] = '\0';
}

static void BuildLabeledValue(char *destination,
                              size_t width,
                              const char *label,
                              const char *value)
{
  const size_t label_length = strlen(label);
  const size_t value_length = strlen(value);
  memset(destination, ' ', width);
  memcpy(destination, label, label_length);
  memcpy(&destination[width - value_length], value, value_length);
  destination[width] = '\0';
}

static bool DrawFixedTextDiff(uint16_t x,
                              uint16_t y,
                              const char *current,
                              const char *previous,
                              uint8_t length,
                              uint8_t scale,
                              uint16_t fg,
                              uint16_t bg,
                              bool force)
{
  const uint16_t advance = (uint16_t)(6U * scale);
  for (uint8_t i = 0U; i < length; ++i)
  {
    if (force || (current[i] != previous[i]))
    {
      if (!LCD_DrawCharScaled(
              (uint16_t)(x + ((uint16_t)i * advance)),
              y,
              current[i],
              scale,
              fg,
              bg))
      {
        return false;
      }
    }
  }
  return true;
}

static void BuildHvText(const DashboardState_t *state,
                        char voltage[HV_VALUE_CHARS + 1U],
                        char bus_current[HV_VALUE_CHARS + 1U],
                        char phase_current[HV_VALUE_CHARS + 1U])
{
  char raw_voltage[20] = "--.- V";
  char raw_bus_current[16] = "--.- A";
  char raw_phase_current[16] = "--.- A";
  const uint8_t part1_valid =
      ((state->detail.flags & BOARD_UART_DETAIL_PART1_VALID) != 0U) ? 1U : 0U;

  if ((state->detail_valid != 0U) && (part1_valid != 0U))
  {
    if (state->detail.bus_voltage_dv <= 9999U)
    {
      FormatTenthsUnsigned(raw_voltage,
                           sizeof(raw_voltage),
                           state->detail.bus_voltage_dv,
                           "V");
    }
    else
    {
      (void)snprintf(raw_voltage, sizeof(raw_voltage), ">999 V");
    }
    FormatTenthsSigned(raw_bus_current,
                       sizeof(raw_bus_current),
                       state->detail.bus_current_da,
                       "A");
    FormatTenthsSigned(raw_phase_current,
                       sizeof(raw_phase_current),
                       state->detail.phase_current_da,
                       "A");
  }
  BuildLabeledValue(voltage, HV_VALUE_CHARS, "VOLT", raw_voltage);
  BuildLabeledValue(bus_current, HV_VALUE_CHARS, "BUS", raw_bus_current);
  BuildLabeledValue(phase_current, HV_VALUE_CHARS, "PHS", raw_phase_current);
}

static void BuildSpeedText(const DashboardState_t *state,
                           char speed[SPEED_CHARS + 1U])
{
  char raw_speed[20] = "--";
  if (state->vehicle_valid != 0U)
  {
    if (SpeedTenthsKmh(state->speed_mps) <= 9999)
    {
      FormatSpeed(raw_speed, sizeof(raw_speed), state->speed_mps);
    }
    else
    {
      (void)snprintf(raw_speed, sizeof(raw_speed), ">999");
    }
  }
  PadLeft(speed, SPEED_CHARS, raw_speed);
}

static void BuildMetricText(const DashboardState_t *state,
                            char motor_temp[METRIC_CHARS + 1U],
                            char controller_temp[METRIC_CHARS + 1U],
                            char fault[METRIC_CHARS + 1U],
                            uint16_t *fault_color)
{
  char raw_motor[16] = "--";
  char raw_controller[16] = "--";
  char raw_fault[16] = "--";
  *fault_color = UI_TEXT;
  const uint8_t part2_valid =
      ((state->detail.flags & BOARD_UART_DETAIL_PART2_VALID) != 0U) ? 1U : 0U;

  if ((state->detail_valid != 0U) && (part2_valid != 0U))
  {
    FormatTenthsSigned(raw_motor,
                       sizeof(raw_motor),
                       state->detail.motor_temperature_dc,
                       "C");
    FormatTenthsSigned(raw_controller,
                       sizeof(raw_controller),
                       state->detail.controller_temperature_dc,
                       "C");
    if (state->detail.motor_fault_bits == 0U)
    {
      (void)snprintf(raw_fault, sizeof(raw_fault), "OK");
      *fault_color = UI_OK;
    }
    else
    {
      (void)snprintf(raw_fault,
                     sizeof(raw_fault),
                     "0X%06lX",
                     (unsigned long)(state->detail.motor_fault_bits & 0xFFFFFFUL));
      *fault_color = UI_FAULT;
    }
  }

  PadLeft(motor_temp, METRIC_CHARS, raw_motor);
  PadLeft(controller_temp, METRIC_CHARS, raw_controller);
  PadLeft(fault, METRIC_CHARS, raw_fault);
}

static const uint16_t badge_x[] = {8U, 120U, 208U, 328U};
static const uint16_t badge_width[] = {104U, 80U, 112U, 144U};

static uint16_t BadgeTextColor(uint16_t color)
{
  return ((color == UI_MANUAL) || (color == UI_OK))
             ? UI_TEXT : LCD_COLOR_WHITE;
}

static void BuildBadgeViews(const DashboardState_t *state,
                            BadgeView_t views[4])
{
  const char *mode_text = "MODE --";
  const char *gear_text = "--";
  uint16_t mode_color = UI_BORDER;
  if (state->vehicle_valid != 0U)
  {
    if (state->mode == 1U)
    {
      mode_text = "AUTO";
      mode_color = UI_AUTO;
    }
    else
    {
      mode_text = "MANUAL";
      mode_color = UI_MANUAL;
    }
    gear_text = (state->gear == 1U) ? "REV" : "FWD";
  }

  const char *comm_text =
      (state->vehicle_valid != 0U) ? "A-B OK" : "A-B LOST";
  const uint16_t comm_color =
      (state->vehicle_valid != 0U) ? UI_OK : UI_FAULT;
  const char *safety_text = "SAFE";
  uint16_t safety_color = UI_OK;
  if (state->estop != 0U)
  {
    safety_text = "E-STOP";
    safety_color = UI_FAULT;
  }
  else if (state->brake != 0U)
  {
    safety_text = "BRAKE";
    safety_color = UI_FAULT;
  }

  views[0].length = 7U;
  views[0].color = mode_color;
  PadRight(views[0].text, views[0].length, mode_text);
  views[1].length = 4U;
  views[1].color = UI_BORDER;
  PadRight(views[1].text, views[1].length, gear_text);
  views[2].length = 8U;
  views[2].color = comm_color;
  PadRight(views[2].text, views[2].length, comm_text);
  views[3].length = 6U;
  views[3].color = safety_color;
  PadRight(views[3].text, views[3].length, safety_text);
}

static bool DrawBadgeView(uint8_t index, const BadgeView_t *view)
{
  return LCD_FillRect(
             badge_x[index], 6U, badge_width[index], 34U, view->color) &&
         LCD_DrawStringScaled((uint16_t)(badge_x[index] + 8U),
                              16U,
                              view->text,
                              2U,
                              BadgeTextColor(view->color),
                              view->color);
}

static bool Dashboard_DrawTopChanges(const DashboardState_t *current,
                                     const DashboardState_t *previous)
{
  BadgeView_t current_views[4];
  BadgeView_t previous_views[4];
  BuildBadgeViews(current, current_views);
  BuildBadgeViews(previous, previous_views);

  for (uint8_t i = 0U; i < 4U; ++i)
  {
    if (current_views[i].color != previous_views[i].color)
    {
      if (!DrawBadgeView(i, &current_views[i])) return false;
    }
    else if (!DrawFixedTextDiff((uint16_t)(badge_x[i] + 8U),
                                16U,
                                current_views[i].text,
                                previous_views[i].text,
                                current_views[i].length,
                                2U,
                                BadgeTextColor(current_views[i].color),
                                current_views[i].color,
                                false))
    {
      return false;
    }
  }
  return true;
}

static bool Dashboard_DrawStaticLayout(void)
{
  return LCD_FillScreen(UI_BG) &&
         LCD_FillRect(8U, 50U, 292U, 146U, UI_CARD) &&
         LCD_DrawRect(8U, 50U, 292U, 146U, 2U, UI_BORDER) &&
         LCD_FillRect(308U, 50U, 164U, 146U, UI_CARD) &&
         LCD_DrawRect(308U, 50U, 164U, 146U, 2U, UI_BORDER) &&
         LCD_FillRect(8U, 204U, 148U, 62U, UI_CARD) &&
         LCD_DrawRect(8U, 204U, 148U, 62U, 2U, UI_BORDER) &&
         LCD_FillRect(166U, 204U, 148U, 62U, UI_CARD) &&
         LCD_DrawRect(166U, 204U, 148U, 62U, 2U, UI_BORDER) &&
         LCD_FillRect(324U, 204U, 148U, 62U, UI_CARD) &&
         LCD_DrawRect(324U, 204U, 148U, 62U, 2U, UI_BORDER) &&
         LCD_DrawStringScaled(18U, 60U, "HV BUS", 2U, UI_TEXT, UI_CARD) &&
         LCD_DrawStringScaled(320U, 60U, "SPEED", 2U, UI_TEXT, UI_CARD) &&
         LCD_DrawStringScaled(18U, 212U, "MOTOR", 2U, UI_TEXT, UI_CARD) &&
         LCD_DrawStringScaled(176U, 212U, "CTRL", 2U, UI_TEXT, UI_CARD) &&
         LCD_DrawStringScaled(334U, 212U, "FAULT", 2U, UI_TEXT, UI_CARD);
}

static bool Dashboard_DrawTop(const DashboardState_t *state)
{
  BadgeView_t views[4];
  BuildBadgeViews(state, views);
  for (uint8_t i = 0U; i < 4U; ++i)
  {
    if (!DrawBadgeView(i, &views[i])) return false;
  }
  return true;
}

static bool Dashboard_DrawHv(const DashboardState_t *state)
{
  char voltage[HV_VALUE_CHARS + 1U];
  char bus_current[HV_VALUE_CHARS + 1U];
  char phase_current[HV_VALUE_CHARS + 1U];
  BuildHvText(state, voltage, bus_current, phase_current);

  return LCD_FillRect(18U, 88U, 270U, 96U, UI_CARD) &&
         LCD_DrawStringScaled(18U, 96U, voltage, 3U, UI_TEXT, UI_CARD) &&
         LCD_DrawStringScaled(18U, 128U, bus_current, 3U, UI_TEXT, UI_CARD) &&
         LCD_DrawStringScaled(18U, 160U, phase_current, 3U, UI_TEXT, UI_CARD);
}

static bool Dashboard_DrawHvChanges(const DashboardState_t *current,
                                    const DashboardState_t *previous)
{
  char current_voltage[HV_VALUE_CHARS + 1U];
  char previous_voltage[HV_VALUE_CHARS + 1U];
  char current_bus[HV_VALUE_CHARS + 1U];
  char previous_bus[HV_VALUE_CHARS + 1U];
  char current_phase[HV_VALUE_CHARS + 1U];
  char previous_phase[HV_VALUE_CHARS + 1U];
  BuildHvText(current, current_voltage, current_bus, current_phase);
  BuildHvText(previous, previous_voltage, previous_bus, previous_phase);

  return DrawFixedTextDiff(18U,
                           96U,
                           current_voltage,
                           previous_voltage,
                           HV_VALUE_CHARS,
                           3U,
                           UI_TEXT,
                           UI_CARD,
                           false) &&
         DrawFixedTextDiff(18U,
                           128U,
                           current_bus,
                           previous_bus,
                           HV_VALUE_CHARS,
                           3U,
                           UI_TEXT,
                           UI_CARD,
                           false) &&
         DrawFixedTextDiff(18U,
                           160U,
                           current_phase,
                           previous_phase,
                           HV_VALUE_CHARS,
                           3U,
                           UI_TEXT,
                           UI_CARD,
                           false);
}

static bool Dashboard_DrawSpeed(const DashboardState_t *state)
{
  char speed[SPEED_CHARS + 1U];
  BuildSpeedText(state, speed);
  return LCD_FillRect(320U, 88U, 140U, 96U, UI_CARD) &&
         LCD_DrawStringScaled(320U, 98U, speed, 5U, UI_TEXT, UI_CARD) &&
         LCD_DrawStringScaled(320U, 158U, "KM/H", 2U, UI_TEXT, UI_CARD);
}

static bool Dashboard_DrawSpeedChanges(const DashboardState_t *current,
                                       const DashboardState_t *previous)
{
  char current_speed[SPEED_CHARS + 1U];
  char previous_speed[SPEED_CHARS + 1U];
  BuildSpeedText(current, current_speed);
  BuildSpeedText(previous, previous_speed);
  return DrawFixedTextDiff(320U,
                           98U,
                           current_speed,
                           previous_speed,
                           SPEED_CHARS,
                           5U,
                           UI_TEXT,
                           UI_CARD,
                           false);
}

static bool Dashboard_DrawMetrics(const DashboardState_t *state)
{
  char motor_temp[METRIC_CHARS + 1U];
  char controller_temp[METRIC_CHARS + 1U];
  char fault[METRIC_CHARS + 1U];
  uint16_t fault_color;
  BuildMetricText(
      state, motor_temp, controller_temp, fault, &fault_color);

  return LCD_FillRect(18U, 236U, 128U, 20U, UI_CARD) &&
         LCD_DrawStringScaled(18U, 238U, motor_temp, 2U, UI_TEXT, UI_CARD) &&
         LCD_FillRect(176U, 236U, 128U, 20U, UI_CARD) &&
         LCD_DrawStringScaled(176U, 238U, controller_temp, 2U, UI_TEXT, UI_CARD) &&
         LCD_FillRect(334U, 236U, 128U, 20U, UI_CARD) &&
         LCD_DrawStringScaled(334U, 238U, fault, 2U, fault_color, UI_CARD);
}

static bool Dashboard_DrawMetricChanges(const DashboardState_t *current,
                                        const DashboardState_t *previous)
{
  char current_motor[METRIC_CHARS + 1U];
  char previous_motor[METRIC_CHARS + 1U];
  char current_controller[METRIC_CHARS + 1U];
  char previous_controller[METRIC_CHARS + 1U];
  char current_fault[METRIC_CHARS + 1U];
  char previous_fault[METRIC_CHARS + 1U];
  uint16_t current_fault_color;
  uint16_t previous_fault_color;
  BuildMetricText(current,
                  current_motor,
                  current_controller,
                  current_fault,
                  &current_fault_color);
  BuildMetricText(previous,
                  previous_motor,
                  previous_controller,
                  previous_fault,
                  &previous_fault_color);

  return DrawFixedTextDiff(18U,
                           238U,
                           current_motor,
                           previous_motor,
                           METRIC_CHARS,
                           2U,
                           UI_TEXT,
                           UI_CARD,
                           false) &&
         DrawFixedTextDiff(176U,
                           238U,
                           current_controller,
                           previous_controller,
                           METRIC_CHARS,
                           2U,
                           UI_TEXT,
                           UI_CARD,
                           false) &&
         DrawFixedTextDiff(334U,
                           238U,
                           current_fault,
                           previous_fault,
                           METRIC_CHARS,
                           2U,
                           current_fault_color,
                           UI_CARD,
                           current_fault_color != previous_fault_color);
}

static void BuildBottomView(const DashboardState_t *state,
                            BottomView_t *view)
{
  char left_text[24];
  char right_text[24];
  uint16_t color = (state->mode == 1U) ? UI_AUTO : UI_MANUAL;

  if (state->estop != 0U)
  {
    (void)snprintf(left_text, sizeof(left_text), "E-STOP ACTIVE");
    (void)snprintf(right_text, sizeof(right_text), "BRAKE ENGAGED");
    color = UI_FAULT;
  }
  else if (state->vehicle_valid == 0U)
  {
    (void)snprintf(left_text, sizeof(left_text), "STM A COMM LOST");
    (void)snprintf(right_text, sizeof(right_text), "VALUES STALE");
    color = UI_FAULT;
  }
  else if ((state->detail_valid != 0U) &&
           ((state->detail.flags & BOARD_UART_DETAIL_PART2_VALID) != 0U) &&
           (state->detail.motor_fault_bits != 0U))
  {
    (void)snprintf(left_text, sizeof(left_text), "MOTOR FAULT");
    (void)snprintf(right_text,
                   sizeof(right_text),
                   "0X%06lX",
                   (unsigned long)(state->detail.motor_fault_bits & 0xFFFFFFUL));
    color = UI_FAULT;
  }
  else if ((state->detail_valid != 0U) &&
           ((state->detail.flags & BOARD_UART_DETAIL_DRIVE_STOP) != 0U))
  {
    (void)snprintf(left_text, sizeof(left_text), "DRIVE STOP");
    (void)snprintf(right_text, sizeof(right_text), "TRACTIVE OFF");
    color = UI_FAULT;
  }
  else if (state->brake != 0U)
  {
    (void)snprintf(left_text, sizeof(left_text), "BRAKE ENGAGED");
    (void)snprintf(right_text, sizeof(right_text), "TRACTIVE STOP");
    color = UI_FAULT;
  }
  else if (state->mode == 1U)
  {
    const uint8_t command_received =
        ((state->detail.flags & BOARD_UART_DETAIL_COMMAND_RECEIVED) != 0U) ? 1U : 0U;
    const uint8_t command_timeout =
        ((state->detail.flags & BOARD_UART_DETAIL_COMMAND_TIMEOUT) != 0U) ? 1U : 0U;
    (void)snprintf(left_text,
                   sizeof(left_text),
                   (state->detail_valid == 0U) ? "AUTO CMD --" :
                   (command_timeout != 0U) ? "AUTO CMD TIMEOUT" :
                   (command_received != 0U) ? "AUTO CMD OK" : "AUTO CMD WAIT");
    (void)snprintf(
        right_text,
        sizeof(right_text),
        (state->detail_valid == 0U) ? "TRACTIVE --" :
        ((state->detail.flags & BOARD_UART_DETAIL_TRACTIVE_FEEDBACK) != 0U)
            ? "TRACTIVE RUN" : "TRACTIVE STOP");
  }
  else
  {
    if (state->detail_valid != 0U)
    {
      (void)snprintf(left_text,
                     sizeof(left_text),
                     "PEDAL %u%%",
                     (unsigned int)state->detail.pedal_percent);
    }
    else
    {
      (void)snprintf(left_text, sizeof(left_text), "PEDAL --");
    }
    (void)snprintf(
        right_text,
        sizeof(right_text),
        (state->detail_valid == 0U) ? "TRACTIVE --" :
        ((state->detail.flags & BOARD_UART_DETAIL_TRACTIVE_FEEDBACK) != 0U)
            ? "TRACTIVE RUN" : "TRACTIVE STOP");
  }

  view->color = color;
  view->text_color =
      (color == UI_MANUAL) ? UI_TEXT : LCD_COLOR_WHITE;
  PadRight(view->left, BOTTOM_LEFT_CHARS, left_text);
  PadRight(view->right, BOTTOM_RIGHT_CHARS, right_text);
}

static bool Dashboard_DrawBottom(const DashboardState_t *state)
{
  BottomView_t view;
  BuildBottomView(state, &view);
  return LCD_FillRect(8U, 274U, 464U, 38U, view.color) &&
         LCD_DrawStringScaled(
             18U, 286U, view.left, 2U, view.text_color, view.color) &&
         LCD_DrawStringScaled(
             270U, 286U, view.right, 2U, view.text_color, view.color);
}

static bool Dashboard_DrawBottomChanges(const DashboardState_t *current,
                                        const DashboardState_t *previous)
{
  BottomView_t current_view;
  BottomView_t previous_view;
  BuildBottomView(current, &current_view);
  BuildBottomView(previous, &previous_view);

  if ((current_view.color != previous_view.color) ||
      (current_view.text_color != previous_view.text_color))
  {
    return Dashboard_DrawBottom(current);
  }
  return DrawFixedTextDiff(18U,
                           286U,
                           current_view.left,
                           previous_view.left,
                           BOTTOM_LEFT_CHARS,
                           2U,
                           current_view.text_color,
                           current_view.color,
                           false) &&
         DrawFixedTextDiff(270U,
                           286U,
                           current_view.right,
                           previous_view.right,
                           BOTTOM_RIGHT_CHARS,
                           2U,
                           current_view.text_color,
                           current_view.color,
                           false);
}

static bool Dashboard_DrawFull(const DashboardState_t *state)
{
  return Dashboard_DrawStaticLayout() &&
         Dashboard_DrawTop(state) &&
         Dashboard_DrawHv(state) &&
         Dashboard_DrawSpeed(state) &&
         Dashboard_DrawMetrics(state) &&
         Dashboard_DrawBottom(state);
}

static bool Dashboard_RefreshBand(const DashboardState_t *state,
                                  uint16_t y,
                                  uint16_t height)
{
  if (!LCD_BeginComposeBand(y, height))
  {
    return false;
  }
  if (!Dashboard_DrawFull(state))
  {
    LCD_CancelComposeBand();
    return false;
  }
  return LCD_EndComposeBand();
}

static bool Dashboard_DrawFullComposed(const DashboardState_t *state)
{
  for (uint16_t y = 0U; y < LCD_LANDSCAPE_HEIGHT;
       y = (uint16_t)(y + LCD_COMPOSE_BAND_MAX_ROWS))
  {
    const uint16_t rows =
        ((uint32_t)y + LCD_COMPOSE_BAND_MAX_ROWS <= LCD_LANDSCAPE_HEIGHT)
            ? LCD_COMPOSE_BAND_MAX_ROWS
            : (uint16_t)(LCD_LANDSCAPE_HEIGHT - y);
    if (!Dashboard_RefreshBand(state, y, rows))
    {
      return false;
    }
  }
  return true;
}

static bool GearResetGesture_Update(GearResetGesture_t *gesture,
                                    const DashboardState_t *state,
                                    uint32_t now)
{
  if (state->vehicle_valid == 0U)
  {
    gesture->transition_count = 0U;
    gesture->initialized = 0U;
    return false;
  }

  const uint8_t gear = state->gear & 0x01U;
  if (gesture->initialized == 0U)
  {
    gesture->previous_gear = gear;
    gesture->initialized = 1U;
    return false;
  }
  if (gear == gesture->previous_gear)
  {
    return false;
  }

  const uint8_t previous_gear = gesture->previous_gear;
  gesture->previous_gear = gear;
  if ((gesture->transition_count != 0U) &&
      ((uint32_t)(now - gesture->started_ms) >
       DISPLAY_GEAR_GESTURE_WINDOW_MS))
  {
    gesture->transition_count = 0U;
  }

  if (gesture->transition_count == 0U)
  {
    if ((previous_gear == 0U) && (gear == 1U))
    {
      gesture->started_ms = now;
      gesture->transition_count = 1U;
    }
    return false;
  }

  gesture->transition_count++;
  if ((gesture->transition_count == 4U) &&
      (gear == 0U) &&
      ((uint32_t)(now - gesture->started_ms) <=
       DISPLAY_GEAR_GESTURE_WINDOW_MS))
  {
    gesture->transition_count = 0U;
    return true;
  }
  return false;
}

/* 모터 상전류 방향이 반전되거나(FWD<->REV 등) 급변하면 그 di/dt가 커패시티브
 * 커플링으로 그라운드 레벨을 흔들어 ST7796S 상태가 깨지고 화면이 하얗게 뜬다.
 * STM_B에는 그라운드를 직접 잴 방법이 없으므로 원인인 상전류를 근거로 삼는다.
 * STM_A가 보내는 detail 프레임의 phase_current_da(0.1A 단위)에서 부호 반전이나
 * 큰 프레임간 변화를 감지하면 교란 시각을 기록하고, 과도 구간이 가라앉도록
 * DISPLAY_PHASE_SETTLE_MS 만큼 기다린 뒤 재초기화 시점에 true를 반환한다.
 * 교란이 연달아 일어나면 마지막 이벤트 기준으로 대기 시간이 다시 시작된다. */
static bool PhaseDisturbanceMonitor_Update(PhaseDisturbanceMonitor_t *monitor,
                                           const DashboardState_t *state,
                                           uint32_t now)
{
  const bool phase_valid =
      (state->detail_valid != 0U) &&
      ((state->detail.flags & BOARD_UART_DETAIL_PART1_VALID) != 0U);

  if (!phase_valid)
  {
    /* 값이 잠깐 무효가 돼도 이미 예약된 재초기화(pending)는 유지한다. */
    monitor->has_previous = 0U;
  }
  else
  {
    const int16_t phase = state->detail.phase_current_da;
    int8_t sign = monitor->phase_sign;
    if (phase >= DISPLAY_PHASE_REVERSAL_DA)
    {
      sign = 1;
    }
    else if (phase <= -DISPLAY_PHASE_REVERSAL_DA)
    {
      sign = -1;
    }

    if (monitor->has_previous != 0U)
    {
      const int32_t delta =
          (int32_t)phase - (int32_t)monitor->previous_phase_da;
      const int32_t magnitude = (delta < 0) ? -delta : delta;
      const bool reversed = (sign != 0) && (monitor->phase_sign != 0) &&
                            (sign != monitor->phase_sign);
      if (reversed || (magnitude >= DISPLAY_PHASE_SWING_DA))
      {
        monitor->event_ms = now;
        monitor->pending = 1U;
      }
    }

    monitor->previous_phase_da = phase;
    monitor->phase_sign = sign;
    monitor->has_previous = 1U;
  }

  if ((monitor->pending != 0U) &&
      ((uint32_t)(now - monitor->event_ms) >= DISPLAY_PHASE_SETTLE_MS))
  {
    monitor->pending = 0U;
    return true;
  }
  return false;
}

static bool DetailPowerChanged(const DashboardState_t *current,
                               const DashboardState_t *previous)
{
  return (current->detail_valid != previous->detail_valid) ||
         (((current->detail.flags ^ previous->detail.flags) &
           BOARD_UART_DETAIL_PART1_VALID) != 0U) ||
         (current->detail.bus_voltage_dv != previous->detail.bus_voltage_dv) ||
         (current->detail.bus_current_da != previous->detail.bus_current_da) ||
         (current->detail.phase_current_da !=
          previous->detail.phase_current_da);
}

static bool DetailThermalChanged(const DashboardState_t *current,
                                 const DashboardState_t *previous)
{
  return (current->detail_valid != previous->detail_valid) ||
         (((current->detail.flags ^ previous->detail.flags) &
           BOARD_UART_DETAIL_PART2_VALID) != 0U) ||
         (current->detail.controller_temperature_dc !=
          previous->detail.controller_temperature_dc) ||
         (current->detail.motor_temperature_dc !=
          previous->detail.motor_temperature_dc) ||
         (current->detail.motor_fault_bits != previous->detail.motor_fault_bits);
}

static bool Dashboard_DrawChanges(const DashboardState_t *current,
                                  const DashboardState_t *previous)
{
  if ((current->mode != previous->mode) ||
      (current->gear != previous->gear) ||
      (current->vehicle_valid != previous->vehicle_valid) ||
      (current->estop != previous->estop) ||
      (current->brake != previous->brake))
  {
    if (!Dashboard_DrawTopChanges(current, previous)) return false;
  }
  if (DetailPowerChanged(current, previous))
  {
    if (!Dashboard_DrawHvChanges(current, previous)) return false;
  }
  if ((SpeedTenthsKmh(current->speed_mps) !=
       SpeedTenthsKmh(previous->speed_mps)) ||
      (current->vehicle_valid != previous->vehicle_valid))
  {
    if (!Dashboard_DrawSpeedChanges(current, previous)) return false;
  }
  if (DetailThermalChanged(current, previous))
  {
    if (!Dashboard_DrawMetricChanges(current, previous)) return false;
  }
  if ((current->mode != previous->mode) ||
      (current->vehicle_valid != previous->vehicle_valid) ||
      (current->detail_valid != previous->detail_valid) ||
      (current->detail.flags != previous->detail.flags) ||
      (current->detail.pedal_percent != previous->detail.pedal_percent) ||
      (current->detail.motor_fault_bits != previous->detail.motor_fault_bits) ||
      (current->estop != previous->estop) ||
      (current->brake != previous->brake))
  {
    if (!Dashboard_DrawBottomChanges(current, previous)) return false;
  }
  return true;
}

static bool Dashboard_Recover(const LCD_Handle_t *lcd,
                              DashboardState_t *displayed_state)
{
  DashboardState_t current;
  LCD_SetBacklight(false);
  if (!LCD_Init(lcd))
  {
    return false;
  }
  Dashboard_ReadState(&current);
  if (!Dashboard_DrawFullComposed(&current))
  {
    return false;
  }
  *displayed_state = current;
  LCD_SetBacklight(true);
  return true;
}

void Dashboard_Task(void *argument)
{
  const LCD_Handle_t lcd = {
      LCD_LANDSCAPE_WIDTH,
      LCD_LANDSCAPE_HEIGHT,
      LCD_ROTATION_LANDSCAPE};
  DashboardState_t displayed_state = {0};
  GearResetGesture_t gear_gesture = {0};
  PhaseDisturbanceMonitor_t phase_monitor = {0};
  uint32_t last_update_ms = 0U;
  uint16_t refresh_y = 0U;
  uint8_t display_ready = 0U;
  (void)argument;

  for (;;)
  {
    const uint32_t now = HAL_GetTick();
    if (display_ready == 0U)
    {
      if (Dashboard_Recover(&lcd, &displayed_state))
      {
        display_ready = 1U;
        last_update_ms = HAL_GetTick();
        refresh_y = 0U;
      }
      else
      {
        LCD_SetBacklight(false);
        osDelay(DISPLAY_RECOVERY_RETRY_MS);
      }
      continue;
    }

    if ((uint32_t)(now - last_update_ms) >= DISPLAY_UPDATE_MS)
    {
      DashboardState_t current;
      Dashboard_ReadState(&current);
      const uint16_t refresh_rows =
          ((uint32_t)refresh_y + LCD_COMPOSE_BAND_MAX_ROWS <=
           LCD_LANDSCAPE_HEIGHT)
              ? LCD_COMPOSE_BAND_MAX_ROWS
              : (uint16_t)(LCD_LANDSCAPE_HEIGHT - refresh_y);
      if (PhaseDisturbanceMonitor_Update(&phase_monitor, &current, now) ||
          GearResetGesture_Update(&gear_gesture, &current, now))
      {
        if (!Dashboard_Recover(&lcd, &displayed_state))
        {
          DISPLAY_RECOVERY_COUNT++;
          DISPLAY_LAST_RECOVERY_ERROR = LCD_LAST_ERROR;
          LCD_SetBacklight(false);
          display_ready = 0U;
        }
        else
        {
          last_update_ms = HAL_GetTick();
          refresh_y = 0U;
        }
        continue;
      }
      if (!Dashboard_DrawChanges(&current, &displayed_state) ||
          !Dashboard_RefreshBand(&current, refresh_y, refresh_rows))
      {
        DISPLAY_RECOVERY_COUNT++;
        DISPLAY_LAST_RECOVERY_ERROR = LCD_LAST_ERROR;
        LCD_SetBacklight(false);
        display_ready = 0U;
        continue;
      }
      displayed_state = current;
      refresh_y = (uint16_t)(refresh_y + refresh_rows);
      if (refresh_y >= LCD_LANDSCAPE_HEIGHT)
      {
        refresh_y = 0U;
      }
      last_update_ms = HAL_GetTick();
    }
    osDelay(5U);
  }
}
