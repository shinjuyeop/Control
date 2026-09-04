#include "manual_tft_ui.h"

#include <Arduino.h>

#include <stdio.h>
#include <string.h>

#include "manual_tft_assets.h"
#include "st7796_lcd.h"

namespace
{

struct Rgb
{
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

enum class HorizontalAnchor : uint8_t
{
  Center,
  Right
};

struct ManualFrameData
{
  bool part1_valid;
  uint8_t bus_progress;
  uint8_t phase_progress;
  Rgb bus_color;
  Rgb phase_color;
  char gear[2];
  char speed[8];
  char bus[10];
  char phase[10];
  char gauge_text[4][8];
  uint8_t gauge_levels[4];
  Rgb gauge_colors[4];
  Rgb gauge_text_colors[4];
  char status_values[6][6];
  Rgb status_colors[6];
  const char *alert;
  Rgb alert_color;
};

static constexpr Rgb COLOR_BLACK = {23U, 24U, 24U};
static constexpr Rgb COLOR_MUTED = {119U, 122U, 122U};
static constexpr Rgb COLOR_WHITE = {255U, 255U, 255U};
static constexpr Rgb COLOR_GEAR = {247U, 247U, 247U};
static constexpr Rgb COLOR_OK = {0U, 168U, 120U};
static constexpr Rgb COLOR_WARN = {242U, 183U, 5U};
static constexpr Rgb COLOR_ORANGE = {242U, 121U, 34U};
static constexpr Rgb COLOR_FAULT = {217U, 45U, 32U};
static constexpr Rgb COLOR_REGEN = {120U, 212U, 244U};

static uint16_t row_pixels[St7796Lcd::WIDTH];
static ManualFrameData displayed_frame = {};

uint16_t toRgb565(Rgb color)
{
  return (uint16_t)(((uint16_t)(color.red >> 3) << 11) |
                    ((uint16_t)(color.green >> 2) << 5) |
                    (color.blue >> 3));
}

Rgb fromRgb565(uint16_t color)
{
  return {
      (uint8_t)((((color >> 11) & 0x1FU) * 255U) / 31U),
      (uint8_t)((((color >> 5) & 0x3FU) * 255U) / 63U),
      (uint8_t)(((color & 0x1FU) * 255U) / 31U)};
}

uint16_t blend(uint16_t background, Rgb foreground, uint8_t alpha)
{
  if (alpha == 0U) return background;
  if (alpha == 255U) return toRgb565(foreground);
  const Rgb base = fromRgb565(background);
  const uint16_t inverse = (uint16_t)(255U - alpha);
  const Rgb mixed = {
      (uint8_t)(((uint16_t)foreground.red * alpha +
                 (uint16_t)base.red * inverse + 127U) / 255U),
      (uint8_t)(((uint16_t)foreground.green * alpha +
                 (uint16_t)base.green * inverse + 127U) / 255U),
      (uint8_t)(((uint16_t)foreground.blue * alpha +
                 (uint16_t)base.blue * inverse + 127U) / 255U)};
  return toRgb565(mixed);
}

void decodeStaticRow(uint16_t y)
{
  const uint32_t begin = manual_static_row_offsets[y];
  const uint32_t end = manual_static_row_offsets[y + 1U];
  uint16_t x = 0U;
  for (uint32_t offset = begin; offset + 2U < end; offset += 3U)
  {
    const uint8_t count = manual_static_row_rle[offset];
    const uint16_t color =
        ((uint16_t)manual_static_row_rle[offset + 1U] << 8) |
        manual_static_row_rle[offset + 2U];
    for (uint8_t index = 0U; index < count; ++index)
    {
      row_pixels[x++] = color;
    }
  }
}

const ManualTftGlyphAsset *findGlyph(const ManualTftFontAsset &font,
                                     uint8_t code)
{
  for (size_t index = 0U; index < font.glyph_count; ++index)
  {
    if (font.glyphs[index].code == code) return &font.glyphs[index];
  }
  return nullptr;
}

uint32_t textAdvanceQ6(const ManualTftFontAsset &font, const char *text)
{
  uint32_t width = 0U;
  while (*text != '\0')
  {
    const ManualTftGlyphAsset *glyph =
        findGlyph(font, (uint8_t)*text++);
    if (glyph != nullptr) width += glyph->advance_q6;
  }
  return width;
}

void drawTextRow(uint16_t y,
                 int16_t anchor_x,
                 int16_t anchor_y,
                 const char *text,
                 const ManualTftFontAsset &font,
                 HorizontalAnchor anchor,
                 Rgb color)
{
  const uint32_t width_q6 = textAdvanceQ6(font, text);
  int32_t cursor_q6 = (int32_t)anchor_x * 64;
  cursor_q6 -= (anchor == HorizontalAnchor::Right)
                   ? (int32_t)width_q6 : (int32_t)(width_q6 / 2U);

  while (*text != '\0')
  {
    const ManualTftGlyphAsset *glyph =
        findGlyph(font, (uint8_t)*text++);
    if (glyph == nullptr) continue;
    const int32_t center_q6 =
        cursor_q6 + ((int32_t)glyph->advance_q6 / 2);
    const int16_t left =
        (int16_t)((center_q6 + 32) / 64) + glyph->left;
    const int16_t top = anchor_y + glyph->top;
    if ((int16_t)y >= top &&
        (int16_t)y < (int16_t)(top + glyph->height))
    {
      const uint32_t alpha_row =
          glyph->alpha_offset +
          (uint32_t)((int16_t)y - top) * glyph->width;
      for (uint8_t x = 0U; x < glyph->width; ++x)
      {
        const int16_t destination_x = left + x;
        if (destination_x >= 0 && destination_x < (int16_t)St7796Lcd::WIDTH)
        {
          row_pixels[destination_x] = blend(
              row_pixels[destination_x], color, font.alpha[alpha_row + x]);
        }
      }
    }
    cursor_q6 += glyph->advance_q6;
  }
}

void fillRowRect(uint16_t y,
                 uint16_t x0,
                 uint16_t y0,
                 uint16_t x1,
                 uint16_t y1,
                 Rgb color)
{
  if (y < y0 || y > y1) return;
  const uint16_t value = toRgb565(color);
  for (uint16_t x = x0; x <= x1 && x < St7796Lcd::WIDTH; ++x)
  {
    row_pixels[x] = value;
  }
}

uint8_t progressFromRange(int32_t value, int32_t minimum, int32_t maximum)
{
  if (value <= minimum) return 0U;
  if (value >= maximum) return 255U;
  return (uint8_t)(((value - minimum) * 255 +
                    ((maximum - minimum) / 2)) /
                   (maximum - minimum));
}

void applyTrack(uint16_t y,
                const ManualTftTrackRow *rows,
                const uint8_t *data,
                uint8_t start,
                uint8_t end,
                Rgb color,
                bool valid)
{
  if (!valid || start == end) return;
  const ManualTftTrackRow row = rows[y];
  if (row.length == 0U) return;
  const uint8_t low = (start < end) ? start : end;
  const uint8_t high = (start > end) ? start : end;
  for (uint16_t index = 0U; index < row.length; ++index)
  {
    const uint8_t alpha = data[row.offset + index * 2U];
    const uint8_t progress = data[row.offset + index * 2U + 1U];
    if (alpha != 0U && progress >= low && progress <= high)
    {
      const uint16_t x = (uint16_t)(row.x + index);
      row_pixels[x] = blend(row_pixels[x], color, alpha);
    }
  }
}

uint8_t gaugeLevel(int32_t value, int32_t minimum, int32_t maximum)
{
  if (value <= minimum) return 0U;
  if (value >= maximum) return 8U;
  return (uint8_t)(((value - minimum) * 8 +
                    ((maximum - minimum) / 2)) /
                   (maximum - minimum));
}

void applyGaugeRows(uint16_t y,
                    const ManualFrameData &frame)
{
  static const uint16_t x_positions[] = {376U, 402U, 428U, 454U};

  for (uint8_t gauge = 0U; gauge < 4U; ++gauge)
  {
    for (uint8_t segment = 0U; segment < 8U; ++segment)
    {
      const uint16_t top = (uint16_t)(94U + segment * 12U);
      if ((uint8_t)(8U - segment) <= frame.gauge_levels[gauge])
      {
        fillRowRect(y, x_positions[gauge], top,
                    (uint16_t)(x_positions[gauge] + 19U),
                    (uint16_t)(top + 8U), frame.gauge_colors[gauge]);
      }
    }
  }
}

void formatSignedTenths(char *buffer, size_t size, int16_t value)
{
  const int32_t signed_value = value;
  const int32_t magnitude = signed_value < 0 ? -signed_value : signed_value;
  (void)snprintf(buffer, size, "%c%ld.%ldA",
                 signed_value < 0 ? '-' : '+',
                 (long)(magnitude / 10),
                 (long)(magnitude % 10));
}

void formatTemperature(char *buffer, size_t size, int16_t value)
{
  const int32_t signed_value = value;
  const int32_t rounded = signed_value < 0
                              ? -((-signed_value + 5) / 10)
                              : ((signed_value + 5) / 10);
  (void)snprintf(buffer, size, "%ld%c", (long)rounded, (char)0xB0);
}

const char *faultTitle(uint32_t bits)
{
  static const char *names[] = {
      "OVERCURRENT", "OVERLOAD", "OVERVOLTAGE", "UNDERVOLTAGE",
      "CTRL OVERHEAT", "MOTOR OVERHEAT", "MOTOR STALLED", "PHASE WIRE",
      "MOTOR SENSOR", "AUX SENSOR", "ENCODER ALIGN", "ANTI RUNAWAY",
      "MAIN THROTTLE", "AUX THROTTLE", "PRECHARGE", "DC CONTACTOR",
      "POWER VALVE", "CURRENT SENSOR", "AUTO TUNE", "RS485",
      "MOTOR CAN", "SOFTWARE", "FAULT 22", "FAULT 23"};
  for (uint8_t index = 0U; index < 24U; ++index)
  {
    if ((bits & (1UL << index)) != 0U) return names[index];
  }
  return "MOTOR FAULT";
}

const char *alertTitle(const DashboardValues &values,
                       bool dashboard_can_lost,
                       bool vehicle_valid,
                       bool detail_valid,
                       bool part1_valid,
                       bool part2_valid,
                       Rgb *color)
{
  *color = COLOR_FAULT;
  if (dashboard_can_lost) return "DASH DATA LOST";
  if ((values.status_flags & STATUS_ESTOP_ACTIVE) != 0U)
    return "E-STOP BRAKING";
  if (!vehicle_valid) return "VEHICLE DATA LOST";
  if (!detail_valid) return "DRIVE DATA LOST";
  if (!part1_valid)
  {
    *color = COLOR_ORANGE;
    return "POWER DATA LOST";
  }
  if (!part2_valid)
  {
    *color = COLOR_ORANGE;
    return "THERMAL DATA LOST";
  }
  if ((values.motor_fault_bits & 0x00FFFFFFUL) != 0U)
    return faultTitle(values.motor_fault_bits);
  if ((values.drive_flags & DRIVE_STOP) != 0U) return "DRIVE STOPPED";
  if ((values.status_flags & STATUS_BRAKE_ACTIVE) != 0U) return "BRAKING";
  return nullptr;
}

bool sameColor(Rgb left, Rgb right)
{
  return left.red == right.red &&
         left.green == right.green &&
         left.blue == right.blue;
}

void buildFrameData(const DashboardValues &values,
                    bool dashboard_can_lost,
                    ManualFrameData *frame)
{
  memset(frame, 0, sizeof(*frame));
  const bool vehicle_valid =
      !dashboard_can_lost &&
      (values.status_flags & STATUS_VEHICLE_VALID) != 0U;
  const bool detail_valid =
      !dashboard_can_lost &&
      (values.status_flags & STATUS_DETAIL_VALID) != 0U;
  const bool part1_valid =
      detail_valid && (values.status_flags & STATUS_PART1_VALID) != 0U;
  const bool part2_valid =
      detail_valid && (values.status_flags & STATUS_PART2_VALID) != 0U;
  frame->part1_valid = part1_valid;

  int32_t phase = values.phase_current_deciamp;
  if (phase < 0) phase = -phase;
  frame->bus_progress = progressFromRange(
      values.bus_current_deciamp, -1200, 3500);
  frame->phase_progress = progressFromRange(phase, 0, 4800);
  frame->bus_color = values.bus_current_deciamp < 0 ? COLOR_REGEN :
      values.bus_current_deciamp <= 1500 ? COLOR_OK :
      values.bus_current_deciamp <= 3000 ? COLOR_WARN : COLOR_FAULT;
  frame->phase_color = phase <= 2880 ? COLOR_OK :
      phase <= 3840 ? COLOR_WARN : COLOR_FAULT;

  frame->gear[0] = '-';
  frame->gear[1] = '\0';
  if (vehicle_valid)
    frame->gear[0] =
        (values.status_flags & STATUS_GEAR_REVERSE) != 0U ? 'R' : 'F';

  (void)snprintf(frame->speed, sizeof(frame->speed), "--.-");
  if (vehicle_valid)
  {
    int32_t magnitude = values.speed_centi_mps;
    if (magnitude < 0) magnitude = -magnitude;
    const uint32_t tenths = ((uint32_t)magnitude * 36U + 50U) / 100U;
    (void)snprintf(frame->speed, sizeof(frame->speed), "%lu.%lu",
                   (unsigned long)(tenths / 10U),
                   (unsigned long)(tenths % 10U));
  }

  (void)snprintf(frame->bus, sizeof(frame->bus), "--.-A");
  (void)snprintf(frame->phase, sizeof(frame->phase), "--.-A");
  (void)snprintf(frame->gauge_text[0], sizeof(frame->gauge_text[0]), "--%%");
  (void)snprintf(frame->gauge_text[1], sizeof(frame->gauge_text[1]), "--.-V");
  (void)snprintf(frame->gauge_text[2], sizeof(frame->gauge_text[2]), "--%c",
                 (char)0xB0);
  (void)snprintf(frame->gauge_text[3], sizeof(frame->gauge_text[3]), "--%c",
                 (char)0xB0);

  if (part1_valid)
  {
    formatSignedTenths(frame->bus, sizeof(frame->bus),
                       values.bus_current_deciamp);
    formatSignedTenths(frame->phase, sizeof(frame->phase),
                       values.phase_current_deciamp);
    (void)snprintf(frame->gauge_text[1], sizeof(frame->gauge_text[1]),
                   "%u.%uV",
                   (unsigned int)(values.bus_voltage_decivolt / 10U),
                   (unsigned int)(values.bus_voltage_decivolt % 10U));
  }
  if (detail_valid)
  {
    (void)snprintf(frame->gauge_text[0], sizeof(frame->gauge_text[0]),
                   "%u%%", (unsigned int)values.pedal_percent);
  }
  if (part2_valid)
  {
    formatTemperature(frame->gauge_text[2], sizeof(frame->gauge_text[2]),
                      values.motor_temperature_decic);
    formatTemperature(frame->gauge_text[3], sizeof(frame->gauge_text[3]),
                      values.controller_temperature_decic);
  }

  frame->gauge_levels[0] = detail_valid
      ? gaugeLevel(values.pedal_percent, 0, 100) : 0U;
  frame->gauge_levels[1] = part1_valid
      ? gaugeLevel(values.bus_voltage_decivolt, 378, 588) : 0U;
  frame->gauge_levels[2] = part2_valid
      ? gaugeLevel(values.motor_temperature_decic, 0, 1300) : 0U;
  frame->gauge_levels[3] = part2_valid
      ? gaugeLevel(values.controller_temperature_decic, 0, 700) : 0U;

  const bool motor_critical =
      part2_valid && (values.motor_fault_bits & (1UL << 5)) != 0U;
  const bool controller_critical =
      part2_valid &&
      (((values.motor_fault_bits & (1UL << 4)) != 0U) ||
       values.controller_temperature_decic >= 700);
  frame->gauge_colors[0] = COLOR_BLACK;
  frame->gauge_colors[1] =
      part1_valid && values.bus_voltage_decivolt < 420U
          ? COLOR_FAULT : COLOR_BLACK;
  frame->gauge_colors[2] =
      motor_critical ? COLOR_FAULT : COLOR_BLACK;
  frame->gauge_colors[3] =
      controller_critical ? COLOR_FAULT : COLOR_BLACK;
  for (uint8_t index = 0U; index < 4U; ++index)
    frame->gauge_text_colors[index] = frame->gauge_colors[index];

  const char *link = "OK";
  Rgb link_color = COLOR_BLACK;
  if (!vehicle_valid)
  {
    link = "LOST";
    link_color = COLOR_FAULT;
  }
  else if (!detail_valid)
  {
    link = "DATA";
    link_color = COLOR_FAULT;
  }
  else if (!part1_valid || !part2_valid)
  {
    link = "PART";
    link_color = COLOR_ORANGE;
  }
  (void)snprintf(frame->status_values[0],
                 sizeof(frame->status_values[0]), "%s", link);
  frame->status_colors[0] = link_color;

  const bool tractive =
      detail_valid && (values.drive_flags & DRIVE_TRACTIVE_FEEDBACK) != 0U;
  (void)snprintf(frame->status_values[1],
                 sizeof(frame->status_values[1]), "%s",
                 tractive ? "ON" : "OFF");
  frame->status_colors[1] = tractive ? COLOR_BLACK : COLOR_MUTED;

  const bool brake = (values.status_flags & STATUS_BRAKE_ACTIVE) != 0U;
  (void)snprintf(frame->status_values[2],
                 sizeof(frame->status_values[2]), "%s",
                 brake ? "ON" : "OFF");
  frame->status_colors[2] = brake ? COLOR_FAULT : COLOR_MUTED;

  const bool estop = (values.status_flags & STATUS_ESTOP_ACTIVE) != 0U;
  (void)snprintf(frame->status_values[3],
                 sizeof(frame->status_values[3]), "%s",
                 estop ? "ON" : "OFF");
  frame->status_colors[3] = estop ? COLOR_FAULT : COLOR_MUTED;

  const uint8_t gnss_state =
      dashboard_can_lost ? GNSS_UNAVAILABLE : values.gnss_state;
  const char *gnss = gnss_state == GNSS_RTK_FIXED ? "FIXED" :
      gnss_state == GNSS_FIX ? "FIX" :
      gnss_state == GNSS_NO_FIX ? "NOFIX" : "--";
  (void)snprintf(frame->status_values[4],
                 sizeof(frame->status_values[4]), "%s", gnss);
  frame->status_colors[4] = gnss_state == GNSS_RTK_FIXED ? COLOR_OK :
      gnss_state == GNSS_FIX ? COLOR_WARN :
      gnss_state == GNSS_NO_FIX ? COLOR_FAULT : COLOR_MUTED;

  const bool fault =
      part2_valid && (values.motor_fault_bits & 0x00FFFFFFUL) != 0U;
  (void)snprintf(frame->status_values[5],
                 sizeof(frame->status_values[5]), "%s",
                 !part2_valid ? "--" : (fault ? "ON" : "OK"));
  frame->status_colors[5] = !part2_valid ? COLOR_MUTED :
      (fault ? COLOR_FAULT : COLOR_BLACK);

  frame->alert = alertTitle(
      values, dashboard_can_lost, vehicle_valid, detail_valid,
      part1_valid, part2_valid, &frame->alert_color);
}

void composeRow(uint16_t y, const ManualFrameData &frame)
{
  static const int16_t gauge_centers[] = {385, 411, 437, 463};
  static const int16_t status_centers[] = {99, 174, 244, 312, 377, 439};

  decodeStaticRow(y);
  applyTrack(y, manual_track_bus_rows, manual_track_bus_data,
             65U, frame.bus_progress, frame.bus_color, frame.part1_valid);
  applyTrack(y, manual_track_phase_rows, manual_track_phase_data,
             0U, frame.phase_progress, frame.phase_color, frame.part1_valid);
  applyGaugeRows(y, frame);
  if (frame.alert != nullptr)
    fillRowRect(y, 281U, 218U, 470U, 242U, frame.alert_color);

  drawTextRow(y, 35, 48, frame.gear, manual_font_gear,
              HorizontalAnchor::Center, COLOR_GEAR);
  const ManualTftFontAsset &speed_font =
      strlen(frame.speed) <= 4U ? manual_font_speed64 : manual_font_speed57;
  drawTextRow(y, 361, 162, frame.speed, speed_font,
              HorizontalAnchor::Right, COLOR_BLACK);
  drawTextRow(y, 126, 229, frame.bus, manual_font_current,
              HorizontalAnchor::Center,
              frame.part1_valid ? COLOR_BLACK : COLOR_MUTED);
  drawTextRow(y, 197, 229, frame.phase, manual_font_current,
              HorizontalAnchor::Center,
              frame.part1_valid ? COLOR_BLACK : COLOR_MUTED);
  for (uint8_t index = 0U; index < 4U; ++index)
  {
    drawTextRow(y, gauge_centers[index], 207, frame.gauge_text[index],
                manual_font_gauge, HorizontalAnchor::Center,
                frame.gauge_text_colors[index]);
  }
  for (uint8_t index = 0U; index < 6U; ++index)
  {
    drawTextRow(y, status_centers[index], 295, frame.status_values[index],
                manual_font_status, HorizontalAnchor::Center,
                frame.status_colors[index]);
  }
  if (frame.alert != nullptr)
  {
    drawTextRow(y, 375, 230, frame.alert, manual_font_alert,
                HorizontalAnchor::Center, COLOR_WHITE);
  }
}

void renderRegion(const ManualFrameData &frame,
                  uint16_t x0,
                  uint16_t y0,
                  uint16_t x1,
                  uint16_t y1)
{
  St7796Lcd::beginPixelStream(x0, y0, x1, y1);
  for (uint16_t y = y0; y <= y1; ++y)
  {
    composeRow(y, frame);
    St7796Lcd::writePixelRow(&row_pixels[x0], (size_t)(x1 - x0 + 1U));
  }
  St7796Lcd::endPixelStream();
}

void renderTrackRows(const ManualFrameData &frame)
{
  for (uint16_t y = 0U; y < St7796Lcd::HEIGHT; ++y)
  {
    const ManualTftTrackRow bus = manual_track_bus_rows[y];
    const ManualTftTrackRow phase = manual_track_phase_rows[y];
    if (bus.length == 0U && phase.length == 0U) continue;

    uint16_t x0 = bus.length != 0U ? bus.x : phase.x;
    uint16_t x1 = bus.length != 0U
        ? (uint16_t)(bus.x + bus.length - 1U)
        : (uint16_t)(phase.x + phase.length - 1U);
    if (phase.length != 0U)
    {
      if (phase.x < x0) x0 = phase.x;
      const uint16_t phase_x1 =
          (uint16_t)(phase.x + phase.length - 1U);
      if (phase_x1 > x1) x1 = phase_x1;
    }

    composeRow(y, frame);
    St7796Lcd::beginPixelStream(x0, y, x1, y);
    St7796Lcd::writePixelRow(&row_pixels[x0], (size_t)(x1 - x0 + 1U));
    St7796Lcd::endPixelStream();
  }
}

void renderFullFrame(const ManualFrameData &frame)
{
  renderRegion(frame, 0U, 0U,
               St7796Lcd::WIDTH - 1U, St7796Lcd::HEIGHT - 1U);
}

bool alertChanged(const ManualFrameData &current,
                  const ManualFrameData &previous)
{
  if ((current.alert == nullptr) != (previous.alert == nullptr)) return true;
  if (current.alert == nullptr) return false;
  return strcmp(current.alert, previous.alert) != 0 ||
         !sameColor(current.alert_color, previous.alert_color);
}

bool statusChanged(const ManualFrameData &current,
                   const ManualFrameData &previous)
{
  for (uint8_t index = 0U; index < 6U; ++index)
  {
    if (strcmp(current.status_values[index],
               previous.status_values[index]) != 0 ||
        !sameColor(current.status_colors[index],
                   previous.status_colors[index]))
      return true;
  }
  return false;
}

void renderChanges(const ManualFrameData &current,
                   const ManualFrameData &previous)
{
  if (current.part1_valid != previous.part1_valid ||
      current.bus_progress != previous.bus_progress ||
      current.phase_progress != previous.phase_progress ||
      !sameColor(current.bus_color, previous.bus_color) ||
      !sameColor(current.phase_color, previous.phase_color))
  {
    renderTrackRows(current);
  }

  if (strcmp(current.gear, previous.gear) != 0)
    renderRegion(current, 4U, 12U, 67U, 82U);
  if (strcmp(current.speed, previous.speed) != 0)
    renderRegion(current, 185U, 130U, 367U, 193U);
  if (strcmp(current.bus, previous.bus) != 0 ||
      strcmp(current.phase, previous.phase) != 0 ||
      current.part1_valid != previous.part1_valid)
  {
    renderRegion(current, 88U, 214U, 238U, 244U);
  }

  static const uint16_t gauge_left[] = {371U, 397U, 423U, 449U};
  static const uint16_t gauge_right[] = {400U, 426U, 452U, 479U};
  for (uint8_t index = 0U; index < 4U; ++index)
  {
    if (strcmp(current.gauge_text[index],
               previous.gauge_text[index]) != 0 ||
        current.gauge_levels[index] != previous.gauge_levels[index] ||
        !sameColor(current.gauge_colors[index],
                   previous.gauge_colors[index]) ||
        !sameColor(current.gauge_text_colors[index],
                   previous.gauge_text_colors[index]))
    {
      renderRegion(current, gauge_left[index], 92U,
                   gauge_right[index], 226U);
    }
  }

  if (statusChanged(current, previous))
    renderRegion(current, 62U, 284U, 471U, 306U);
  if (alertChanged(current, previous))
    renderRegion(current, 281U, 218U, 470U, 242U);
}
}  // namespace

namespace ManualTftUi
{

void playStartup()
{
  for (size_t index = 0U;
       index < sizeof(manual_startup_frames) / sizeof(manual_startup_frames[0]);
       ++index)
  {
    St7796Lcd::drawRleImage(
        manual_startup_frames[index], manual_startup_sizes[index]);
  }
}

void begin(const DashboardValues &values, bool dashboard_can_lost)
{
  ManualFrameData current = {};
  buildFrameData(values, dashboard_can_lost, &current);
  renderFullFrame(current);
  displayed_frame = current;
}

void update(const DashboardValues &values, bool dashboard_can_lost)
{
  ManualFrameData current = {};
  buildFrameData(values, dashboard_can_lost, &current);
  renderChanges(current, displayed_frame);
  displayed_frame = current;
}

}  // namespace ManualTftUi
