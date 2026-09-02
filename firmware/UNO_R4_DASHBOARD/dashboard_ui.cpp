#include "dashboard_ui.h"

#include <stdio.h>
#include <string.h>

#include "st7796_lcd.h"

namespace
{

static constexpr uint16_t UI_BG = St7796Lcd::COLOR_WHITE;
static constexpr uint16_t UI_TEXT = St7796Lcd::COLOR_BLACK;
static constexpr uint16_t UI_CARD = St7796Lcd::COLOR_LIGHT_GRAY;
static constexpr uint16_t UI_BORDER = St7796Lcd::COLOR_DARK_GRAY;
static constexpr uint16_t UI_AUTO = St7796Lcd::COLOR_BLUE;
static constexpr uint16_t UI_MANUAL = St7796Lcd::COLOR_ORANGE;
static constexpr uint16_t UI_OK = St7796Lcd::COLOR_GREEN;
static constexpr uint16_t UI_FAULT = St7796Lcd::COLOR_RED;

static constexpr size_t HV_VALUE_CHARS = 14U;
static constexpr size_t SPEED_CHARS = 5U;
static constexpr size_t METRIC_CHARS = 8U;
static constexpr size_t BOTTOM_LEFT_CHARS = 20U;
static constexpr size_t BOTTOM_RIGHT_CHARS = 16U;

struct BadgeView
{
  char text[9];
  uint16_t color;
  uint8_t length;
};

struct BottomView
{
  char left[BOTTOM_LEFT_CHARS + 1U];
  char right[BOTTOM_RIGHT_CHARS + 1U];
  uint16_t color;
  uint16_t text_color;
};

struct UiView
{
  BadgeView badges[4];
  char voltage[HV_VALUE_CHARS + 1U];
  char bus_current[HV_VALUE_CHARS + 1U];
  char phase_current[HV_VALUE_CHARS + 1U];
  char speed[SPEED_CHARS + 1U];
  char motor_temperature[METRIC_CHARS + 1U];
  char controller_temperature[METRIC_CHARS + 1U];
  char fault[METRIC_CHARS + 1U];
  uint16_t fault_color;
  BottomView bottom;
};

static const uint16_t badge_x[] = {8U, 120U, 208U, 328U};
static const uint16_t badge_width[] = {104U, 80U, 112U, 144U};
static UiView displayed_view = {};
static bool initialized = false;

bool flagIsSet(uint8_t value, uint8_t flag)
{
  return (value & flag) != 0U;
}

void formatTenthsSigned(char *buffer,
                        size_t size,
                        int16_t value,
                        const char *unit)
{
  const int32_t signed_value = value;
  const int32_t magnitude =
      (signed_value < 0) ? -signed_value : signed_value;
  (void)snprintf(buffer,
                 size,
                 "%s%ld.%ld %s",
                 (signed_value < 0) ? "-" : "",
                 (long)(magnitude / 10),
                 (long)(magnitude % 10),
                 unit);
}

void formatTenthsUnsigned(char *buffer,
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

int32_t speedTenthsKmh(int16_t speed_centi_mps)
{
  const int32_t signed_speed = speed_centi_mps;
  const uint32_t magnitude =
      (signed_speed < 0) ? (uint32_t)(-signed_speed)
                         : (uint32_t)signed_speed;
  return (int32_t)(((magnitude * 36U) + 50U) / 100U);
}

void formatSpeed(char *buffer, size_t size, int16_t speed_centi_mps)
{
  const int32_t tenths = speedTenthsKmh(speed_centi_mps);
  (void)snprintf(buffer,
                 size,
                 "%ld.%ld",
                 (long)(tenths / 10),
                 (long)(tenths % 10));
}

void padLeft(char *destination, size_t width, const char *source)
{
  size_t length = strlen(source);
  if (length > width) length = width;
  const size_t padding = width - length;
  memset(destination, ' ', padding);
  memcpy(&destination[padding], source, length);
  destination[width] = '\0';
}

void padRight(char *destination, size_t width, const char *source)
{
  size_t length = strlen(source);
  if (length > width) length = width;
  memcpy(destination, source, length);
  memset(&destination[length], ' ', width - length);
  destination[width] = '\0';
}

void buildLabeledValue(char *destination,
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

uint16_t badgeTextColor(uint16_t color)
{
  return ((color == UI_MANUAL) || (color == UI_OK))
             ? UI_TEXT : St7796Lcd::COLOR_WHITE;
}

void buildBadges(const DashboardValues &values, BadgeView views[4])
{
  const bool vehicle_valid =
      flagIsSet(values.status_flags, STATUS_VEHICLE_VALID);
  const char *mode_text = "MODE --";
  const char *gear_text = "--";
  uint16_t mode_color = UI_BORDER;
  if (vehicle_valid)
  {
    if (flagIsSet(values.status_flags, STATUS_MODE_AUTO))
    {
      mode_text = "AUTO";
      mode_color = UI_AUTO;
    }
    else
    {
      mode_text = "MANUAL";
      mode_color = UI_MANUAL;
    }
    gear_text = flagIsSet(values.status_flags, STATUS_GEAR_REVERSE)
                    ? "REV" : "FWD";
  }

  const char *comm_text = vehicle_valid ? "A-B OK" : "A-B LOST";
  const uint16_t comm_color = vehicle_valid ? UI_OK : UI_FAULT;
  const char *safety_text = "SAFE";
  uint16_t safety_color = UI_OK;
  if (flagIsSet(values.status_flags, STATUS_ESTOP_ACTIVE))
  {
    safety_text = "E-STOP";
    safety_color = UI_FAULT;
  }
  else if (flagIsSet(values.status_flags, STATUS_BRAKE_ACTIVE))
  {
    safety_text = "BRAKE";
    safety_color = UI_FAULT;
  }

  views[0].length = 7U;
  views[0].color = mode_color;
  padRight(views[0].text, views[0].length, mode_text);
  views[1].length = 4U;
  views[1].color = UI_BORDER;
  padRight(views[1].text, views[1].length, gear_text);
  views[2].length = 8U;
  views[2].color = comm_color;
  padRight(views[2].text, views[2].length, comm_text);
  views[3].length = 6U;
  views[3].color = safety_color;
  padRight(views[3].text, views[3].length, safety_text);
}

void buildPower(const DashboardValues &values,
                char voltage[HV_VALUE_CHARS + 1U],
                char bus_current[HV_VALUE_CHARS + 1U],
                char phase_current[HV_VALUE_CHARS + 1U])
{
  char raw_voltage[20] = "--.- V";
  char raw_bus_current[16] = "--.- A";
  char raw_phase_current[16] = "--.- A";
  const bool part1_valid =
      flagIsSet(values.status_flags, STATUS_DETAIL_VALID) &&
      flagIsSet(values.status_flags, STATUS_PART1_VALID);

  if (part1_valid)
  {
    if (values.bus_voltage_decivolt <= 9999U)
    {
      formatTenthsUnsigned(raw_voltage,
                           sizeof(raw_voltage),
                           values.bus_voltage_decivolt,
                           "V");
    }
    else
    {
      (void)snprintf(raw_voltage, sizeof(raw_voltage), ">999 V");
    }
    formatTenthsSigned(raw_bus_current,
                       sizeof(raw_bus_current),
                       values.bus_current_deciamp,
                       "A");
    formatTenthsSigned(raw_phase_current,
                       sizeof(raw_phase_current),
                       values.phase_current_deciamp,
                       "A");
  }

  buildLabeledValue(voltage, HV_VALUE_CHARS, "VOLT", raw_voltage);
  buildLabeledValue(bus_current, HV_VALUE_CHARS, "BUS", raw_bus_current);
  buildLabeledValue(phase_current, HV_VALUE_CHARS, "PHS", raw_phase_current);
}

void buildSpeed(const DashboardValues &values,
                char speed[SPEED_CHARS + 1U])
{
  char raw_speed[20] = "--";
  if (flagIsSet(values.status_flags, STATUS_VEHICLE_VALID))
  {
    if (speedTenthsKmh(values.speed_centi_mps) <= 9999)
    {
      formatSpeed(raw_speed, sizeof(raw_speed), values.speed_centi_mps);
    }
    else
    {
      (void)snprintf(raw_speed, sizeof(raw_speed), ">999");
    }
  }
  padLeft(speed, SPEED_CHARS, raw_speed);
}

void buildMetrics(const DashboardValues &values,
                  char motor_temperature[METRIC_CHARS + 1U],
                  char controller_temperature[METRIC_CHARS + 1U],
                  char fault[METRIC_CHARS + 1U],
                  uint16_t *fault_color)
{
  char raw_motor[16] = "--";
  char raw_controller[16] = "--";
  char raw_fault[16] = "--";
  *fault_color = UI_TEXT;
  const bool part2_valid =
      flagIsSet(values.status_flags, STATUS_DETAIL_VALID) &&
      flagIsSet(values.status_flags, STATUS_PART2_VALID);

  if (part2_valid)
  {
    formatTenthsSigned(raw_motor,
                       sizeof(raw_motor),
                       values.motor_temperature_decic,
                       "C");
    formatTenthsSigned(raw_controller,
                       sizeof(raw_controller),
                       values.controller_temperature_decic,
                       "C");
    if ((values.motor_fault_bits & 0x00FFFFFFUL) == 0U)
    {
      (void)snprintf(raw_fault, sizeof(raw_fault), "OK");
      *fault_color = UI_OK;
    }
    else
    {
      (void)snprintf(raw_fault,
                     sizeof(raw_fault),
                     "0X%06lX",
                     (unsigned long)(values.motor_fault_bits & 0x00FFFFFFUL));
      *fault_color = UI_FAULT;
    }
  }

  padLeft(motor_temperature, METRIC_CHARS, raw_motor);
  padLeft(controller_temperature, METRIC_CHARS, raw_controller);
  padLeft(fault, METRIC_CHARS, raw_fault);
}

void buildBottom(const DashboardValues &values,
                 bool dashboard_can_lost,
                 BottomView *view)
{
  char left_text[24];
  char right_text[24];
  const bool vehicle_valid =
      flagIsSet(values.status_flags, STATUS_VEHICLE_VALID);
  const bool detail_valid =
      flagIsSet(values.status_flags, STATUS_DETAIL_VALID);
  const bool mode_auto = flagIsSet(values.status_flags, STATUS_MODE_AUTO);
  uint16_t color = mode_auto ? UI_AUTO : UI_MANUAL;

  if (dashboard_can_lost)
  {
    (void)snprintf(left_text, sizeof(left_text), "DASH CAN LOST");
    (void)snprintf(right_text, sizeof(right_text), "VALUES STALE");
    color = UI_FAULT;
  }
  else if (flagIsSet(values.status_flags, STATUS_ESTOP_ACTIVE))
  {
    (void)snprintf(left_text, sizeof(left_text), "E-STOP ACTIVE");
    (void)snprintf(right_text, sizeof(right_text), "BRAKE ENGAGED");
    color = UI_FAULT;
  }
  else if (!vehicle_valid)
  {
    (void)snprintf(left_text, sizeof(left_text), "STM A COMM LOST");
    (void)snprintf(right_text, sizeof(right_text), "VALUES STALE");
    color = UI_FAULT;
  }
  else if (detail_valid &&
           flagIsSet(values.status_flags, STATUS_PART2_VALID) &&
           ((values.motor_fault_bits & 0x00FFFFFFUL) != 0U))
  {
    (void)snprintf(left_text, sizeof(left_text), "MOTOR FAULT");
    (void)snprintf(right_text,
                   sizeof(right_text),
                   "0X%06lX",
                   (unsigned long)(values.motor_fault_bits & 0x00FFFFFFUL));
    color = UI_FAULT;
  }
  else if (detail_valid && flagIsSet(values.drive_flags, DRIVE_STOP))
  {
    (void)snprintf(left_text, sizeof(left_text), "DRIVE STOP");
    (void)snprintf(right_text, sizeof(right_text), "TRACTIVE OFF");
    color = UI_FAULT;
  }
  else if (flagIsSet(values.status_flags, STATUS_BRAKE_ACTIVE))
  {
    (void)snprintf(left_text, sizeof(left_text), "BRAKE ENGAGED");
    (void)snprintf(right_text, sizeof(right_text), "TRACTIVE STOP");
    color = UI_FAULT;
  }
  else if (mode_auto)
  {
    (void)snprintf(
        left_text,
        sizeof(left_text),
        !detail_valid ? "AUTO CMD --" :
        flagIsSet(values.drive_flags, DRIVE_COMMAND_TIMEOUT)
            ? "AUTO CMD TIMEOUT" :
        flagIsSet(values.drive_flags, DRIVE_COMMAND_RECEIVED)
            ? "AUTO CMD OK" : "AUTO CMD WAIT");
    (void)snprintf(
        right_text,
        sizeof(right_text),
        !detail_valid ? "TRACTIVE --" :
        flagIsSet(values.drive_flags, DRIVE_TRACTIVE_FEEDBACK)
            ? "TRACTIVE RUN" : "TRACTIVE STOP");
  }
  else
  {
    if (detail_valid)
    {
      (void)snprintf(left_text,
                     sizeof(left_text),
                     "PEDAL %u%%",
                     (unsigned int)values.pedal_percent);
    }
    else
    {
      (void)snprintf(left_text, sizeof(left_text), "PEDAL --");
    }
    (void)snprintf(
        right_text,
        sizeof(right_text),
        !detail_valid ? "TRACTIVE --" :
        flagIsSet(values.drive_flags, DRIVE_TRACTIVE_FEEDBACK)
            ? "TRACTIVE RUN" : "TRACTIVE STOP");
  }

  view->color = color;
  view->text_color =
      (color == UI_MANUAL) ? UI_TEXT : St7796Lcd::COLOR_WHITE;
  padRight(view->left, BOTTOM_LEFT_CHARS, left_text);
  padRight(view->right, BOTTOM_RIGHT_CHARS, right_text);
}

void buildView(const DashboardValues &values,
               bool dashboard_can_lost,
               UiView *view)
{
  buildBadges(values, view->badges);
  buildPower(values,
             view->voltage,
             view->bus_current,
             view->phase_current);
  buildSpeed(values, view->speed);
  buildMetrics(values,
               view->motor_temperature,
               view->controller_temperature,
               view->fault,
               &view->fault_color);
  buildBottom(values, dashboard_can_lost, &view->bottom);
}

void drawFixedTextDiff(uint16_t x,
                       uint16_t y,
                       const char *current,
                       const char *previous,
                       size_t length,
                       uint8_t scale,
                       uint16_t foreground,
                       uint16_t background,
                       bool force)
{
  const uint16_t advance = (uint16_t)(6U * scale);
  for (size_t i = 0U; i < length; ++i)
  {
    if (force || (current[i] != previous[i]))
    {
      St7796Lcd::drawCharScaled(
          (uint16_t)(x + ((uint16_t)i * advance)),
          y,
          current[i],
          scale,
          foreground,
          background);
    }
  }
}

void drawBadge(uint8_t index, const BadgeView &view)
{
  St7796Lcd::fillRect(
      badge_x[index], 6U, badge_width[index], 34U, view.color);
  St7796Lcd::drawStringScaled((uint16_t)(badge_x[index] + 8U),
                             16U,
                             view.text,
                             2U,
                             badgeTextColor(view.color),
                             view.color);
}

void drawStaticLayout()
{
  St7796Lcd::fillScreen(UI_BG);
  St7796Lcd::fillRect(8U, 50U, 292U, 146U, UI_CARD);
  St7796Lcd::drawRect(8U, 50U, 292U, 146U, 2U, UI_BORDER);
  St7796Lcd::fillRect(308U, 50U, 164U, 146U, UI_CARD);
  St7796Lcd::drawRect(308U, 50U, 164U, 146U, 2U, UI_BORDER);
  St7796Lcd::fillRect(8U, 204U, 148U, 62U, UI_CARD);
  St7796Lcd::drawRect(8U, 204U, 148U, 62U, 2U, UI_BORDER);
  St7796Lcd::fillRect(166U, 204U, 148U, 62U, UI_CARD);
  St7796Lcd::drawRect(166U, 204U, 148U, 62U, 2U, UI_BORDER);
  St7796Lcd::fillRect(324U, 204U, 148U, 62U, UI_CARD);
  St7796Lcd::drawRect(324U, 204U, 148U, 62U, 2U, UI_BORDER);
  St7796Lcd::drawStringScaled(18U, 60U, "HV BUS", 2U, UI_TEXT, UI_CARD);
  St7796Lcd::drawStringScaled(320U, 60U, "SPEED", 2U, UI_TEXT, UI_CARD);
  St7796Lcd::drawStringScaled(18U, 212U, "MOTOR", 2U, UI_TEXT, UI_CARD);
  St7796Lcd::drawStringScaled(176U, 212U, "CTRL", 2U, UI_TEXT, UI_CARD);
  St7796Lcd::drawStringScaled(334U, 212U, "FAULT", 2U, UI_TEXT, UI_CARD);
}

void drawFullView(const UiView &view)
{
  drawStaticLayout();
  for (uint8_t i = 0U; i < 4U; ++i) drawBadge(i, view.badges[i]);

  St7796Lcd::fillRect(18U, 88U, 270U, 96U, UI_CARD);
  St7796Lcd::drawStringScaled(
      18U, 96U, view.voltage, 3U, UI_TEXT, UI_CARD);
  St7796Lcd::drawStringScaled(
      18U, 128U, view.bus_current, 3U, UI_TEXT, UI_CARD);
  St7796Lcd::drawStringScaled(
      18U, 160U, view.phase_current, 3U, UI_TEXT, UI_CARD);

  St7796Lcd::fillRect(320U, 88U, 140U, 96U, UI_CARD);
  St7796Lcd::drawStringScaled(
      320U, 98U, view.speed, 5U, UI_TEXT, UI_CARD);
  St7796Lcd::drawStringScaled(
      320U, 158U, "KM/H", 2U, UI_TEXT, UI_CARD);

  St7796Lcd::fillRect(18U, 236U, 128U, 20U, UI_CARD);
  St7796Lcd::drawStringScaled(
      18U, 238U, view.motor_temperature, 2U, UI_TEXT, UI_CARD);
  St7796Lcd::fillRect(176U, 236U, 128U, 20U, UI_CARD);
  St7796Lcd::drawStringScaled(
      176U, 238U, view.controller_temperature, 2U, UI_TEXT, UI_CARD);
  St7796Lcd::fillRect(334U, 236U, 128U, 20U, UI_CARD);
  St7796Lcd::drawStringScaled(
      334U, 238U, view.fault, 2U, view.fault_color, UI_CARD);

  St7796Lcd::fillRect(8U, 274U, 464U, 38U, view.bottom.color);
  St7796Lcd::drawStringScaled(18U,
                             286U,
                             view.bottom.left,
                             2U,
                             view.bottom.text_color,
                             view.bottom.color);
  St7796Lcd::drawStringScaled(270U,
                             286U,
                             view.bottom.right,
                             2U,
                             view.bottom.text_color,
                             view.bottom.color);
}

void drawViewChanges(const UiView &current, const UiView &previous)
{
  for (uint8_t i = 0U; i < 4U; ++i)
  {
    if (current.badges[i].color != previous.badges[i].color)
    {
      drawBadge(i, current.badges[i]);
    }
    else
    {
      drawFixedTextDiff((uint16_t)(badge_x[i] + 8U),
                        16U,
                        current.badges[i].text,
                        previous.badges[i].text,
                        current.badges[i].length,
                        2U,
                        badgeTextColor(current.badges[i].color),
                        current.badges[i].color,
                        false);
    }
  }

  drawFixedTextDiff(18U, 96U, current.voltage, previous.voltage,
                    HV_VALUE_CHARS, 3U, UI_TEXT, UI_CARD, false);
  drawFixedTextDiff(18U, 128U, current.bus_current, previous.bus_current,
                    HV_VALUE_CHARS, 3U, UI_TEXT, UI_CARD, false);
  drawFixedTextDiff(18U, 160U, current.phase_current, previous.phase_current,
                    HV_VALUE_CHARS, 3U, UI_TEXT, UI_CARD, false);
  drawFixedTextDiff(320U, 98U, current.speed, previous.speed,
                    SPEED_CHARS, 5U, UI_TEXT, UI_CARD, false);
  drawFixedTextDiff(18U, 238U,
                    current.motor_temperature,
                    previous.motor_temperature,
                    METRIC_CHARS, 2U, UI_TEXT, UI_CARD, false);
  drawFixedTextDiff(176U, 238U,
                    current.controller_temperature,
                    previous.controller_temperature,
                    METRIC_CHARS, 2U, UI_TEXT, UI_CARD, false);
  drawFixedTextDiff(334U, 238U,
                    current.fault,
                    previous.fault,
                    METRIC_CHARS,
                    2U,
                    current.fault_color,
                    UI_CARD,
                    current.fault_color != previous.fault_color);

  if ((current.bottom.color != previous.bottom.color) ||
      (current.bottom.text_color != previous.bottom.text_color))
  {
    St7796Lcd::fillRect(8U, 274U, 464U, 38U, current.bottom.color);
    St7796Lcd::drawStringScaled(18U,
                               286U,
                               current.bottom.left,
                               2U,
                               current.bottom.text_color,
                               current.bottom.color);
    St7796Lcd::drawStringScaled(270U,
                               286U,
                               current.bottom.right,
                               2U,
                               current.bottom.text_color,
                               current.bottom.color);
  }
  else
  {
    drawFixedTextDiff(18U, 286U,
                      current.bottom.left,
                      previous.bottom.left,
                      BOTTOM_LEFT_CHARS,
                      2U,
                      current.bottom.text_color,
                      current.bottom.color,
                      false);
    drawFixedTextDiff(270U, 286U,
                      current.bottom.right,
                      previous.bottom.right,
                      BOTTOM_RIGHT_CHARS,
                      2U,
                      current.bottom.text_color,
                      current.bottom.color,
                      false);
  }
}

}  // namespace

namespace DashboardUi
{

void begin(const DashboardValues &values, bool dashboard_can_lost)
{
  UiView view = {};
  buildView(values, dashboard_can_lost, &view);
  St7796Lcd::setBacklight(false);
  drawFullView(view);
  displayed_view = view;
  initialized = true;
  St7796Lcd::setBacklight(true);
}

void update(const DashboardValues &values, bool dashboard_can_lost)
{
  if (!initialized)
  {
    begin(values, dashboard_can_lost);
    return;
  }
  UiView current = {};
  buildView(values, dashboard_can_lost, &current);
  drawViewChanges(current, displayed_view);
  displayed_view = current;
}

}  // namespace DashboardUi
