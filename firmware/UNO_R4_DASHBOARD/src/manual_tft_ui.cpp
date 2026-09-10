#include "include/manual_tft_ui.h"

#include <Arduino.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "assets/manual_tft_assets.h"
#include "include/dashboard_timing.h"
#include "assets/startup_tft_assets.h"
#include "include/st7796_lcd.h"

namespace
{

struct Rgb
{
  uint8_t red;
  uint8_t green;
  uint8_t blue;
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
  int16_t dino_obstacle_x;
  uint8_t dino_jump_px;
  uint8_t dino_foot_frame;
  uint8_t dino_ground_scroll;
  bool dino_visible;
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
static constexpr Rgb COLOR_DINO = {138U, 142U, 142U};
static constexpr Rgb COLOR_DINO_GROUND = {184U, 187U, 187U};

static constexpr uint16_t DINO_X = 300U;
static constexpr uint16_t DINO_RIGHT_X = 324U;
static constexpr uint16_t DINO_GROUND_Y = 256U;
static constexpr uint16_t DINO_REGION_X0 = 281U;
static constexpr uint16_t DINO_REGION_Y0 = 206U;
static constexpr uint16_t DINO_REGION_X1 = 470U;
static constexpr uint16_t DINO_REGION_Y1 = 257U;
static constexpr float DINO_PIXELS_PER_M = 8.0f;
static constexpr float DINO_OBSTACLE_CYCLE_PX = 210.0f;
static constexpr float DINO_OBSTACLE_OFFSET_PX = 60.0f;
static constexpr float DINO_JUMP_TRIGGER_GAP_PX = 20.0f;
static constexpr float DINO_JUMP_DISTANCE_PX = 45.0f;
static constexpr float DINO_JUMP_HEIGHT_PX = 28.0f;
static constexpr uint32_t DINO_JUMP_DELAY_MS = 100U;
static constexpr uint16_t DIRTY_ALERT = 1U << 0;
static constexpr uint16_t DIRTY_SPEED = 1U << 1;
static constexpr uint16_t DIRTY_CURRENT = 1U << 2;
static constexpr uint16_t DIRTY_PEDAL = 1U << 3;
static constexpr uint16_t DIRTY_HV = 1U << 4;
static constexpr uint16_t DIRTY_MOTOR = 1U << 5;
static constexpr uint16_t DIRTY_CONTROLLER = 1U << 6;
static constexpr uint16_t DIRTY_GEAR = 1U << 7;
static constexpr uint16_t DIRTY_STATUS = 1U << 8;
static constexpr uint16_t DIRTY_TRACK = 1U << 9;
static constexpr uint16_t DIRTY_DINO = 1U << 10;
static constexpr uint16_t URGENT_REGIONS =
    DIRTY_ALERT | DIRTY_STATUS | DIRTY_DINO;
static constexpr uint8_t RENDER_ROWS_PER_SERVICE = 4U;
static constexpr uint32_t STARTUP_DURATION_MS = 1400U;
static constexpr uint32_t STARTUP_REVEAL_END_MS = 350U;
static constexpr uint32_t STARTUP_HOLD_END_MS = 1100U;

static const uint32_t DINO_RUN_FRAMES[2][22] = {
    {0x3FC000UL, 0x7FE000UL, 0x7CE000UL, 0x7CE000UL, 0x7FE000UL, 0x03E000UL,
     0x1FF000UL, 0x03F800UL, 0x03FC00UL, 0x0FFE01UL, 0x0DFF03UL, 0x01FF87UL,
     0x01FFCFUL, 0x01FFFFUL, 0x01FFFEUL, 0x01FFFCUL, 0x00FFF0UL, 0x007FC0UL,
     0x007380UL, 0x00E180UL, 0x018180UL, 0x000780UL},
    {0x3FC000UL, 0x7FE000UL, 0x7CE000UL, 0x7CE000UL, 0x7FE000UL, 0x03E000UL,
     0x1FF000UL, 0x03F800UL, 0x03FC00UL, 0x0FFE01UL, 0x0DFF03UL, 0x01FF87UL,
     0x01FFCFUL, 0x01FFFFUL, 0x01FFFEUL, 0x01FFFCUL, 0x00FFF0UL, 0x007FC0UL,
     0x007380UL, 0x003700UL, 0x003600UL, 0x00F000UL},
};
static const uint8_t DINO_CACTUS[8] = {
    0x2U, 0x2U, 0x3U, 0x6U, 0x2U, 0x2U, 0x2U, 0x7U};

static uint16_t row_pixels[St7796Lcd::WIDTH];
static ManualFrameData desired_frame = {};
static ManualFrameData previous_queued_frame = {};
static ManualFrameData active_frame = {};
static bool have_queued_frame = false;
static uint16_t pending_regions = 0U;
static uint16_t urgent_regions = 0U;
static uint16_t active_region = 0U;
static uint16_t active_y = 0U;
static uint8_t round_robin_index = 0U;
static bool rendered_track_valid = false;
static bool active_track_full = false;
static uint8_t rendered_bus_progress = 0U;
static uint8_t rendered_phase_progress = 0U;
static Rgb rendered_bus_color = {};
static Rgb rendered_phase_color = {};
static bool rendered_part1_valid = false;
static float dino_distance_px = 0.0f;
static float dino_jump_start_px = -1.0f;
static uint32_t dino_jump_delay_ms = 0U;
static uint32_t dino_last_update_ms = 0U;
static bool dino_time_initialized = false;

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

void drawTextRow(uint16_t y,
                 int16_t anchor_x,
                 int16_t anchor_y,
                 const char *text,
                 const TftFontAsset &font,
                 TftHorizontalAnchor anchor,
                 Rgb color)
{
  TftFontRenderer::drawTextRow(
      row_pixels, St7796Lcd::WIDTH, y, anchor_x, anchor_y,
      text, font, toRgb565(color), anchor);
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

int16_t dinoObstacleX()
{
  const float travel = fmodf(
      dino_distance_px + DINO_OBSTACLE_OFFSET_PX,
      DINO_OBSTACLE_CYCLE_PX);
  return (int16_t)lroundf(470.0f - travel);
}

uint8_t dinoJumpPx()
{
  if (dino_jump_start_px < 0.0f || dino_jump_delay_ms != 0U) return 0U;
  float progress =
      (dino_distance_px - dino_jump_start_px) / DINO_JUMP_DISTANCE_PX;
  if (progress < 0.0f) progress = 0.0f;
  if (progress > 1.0f) progress = 1.0f;
  return (uint8_t)lroundf(
      4.0f * progress * (1.0f - progress) * DINO_JUMP_HEIGHT_PX);
}

void advanceDino(const DashboardValues &values,
                 bool dashboard_can_lost,
                 uint32_t now_ms)
{
  if (!dino_time_initialized)
  {
    dino_last_update_ms = now_ms;
    dino_time_initialized = true;
    return;
  }
  uint32_t elapsed_ms = (uint32_t)(now_ms - dino_last_update_ms);
  dino_last_update_ms = now_ms;
  if (elapsed_ms > 200U) elapsed_ms = 200U;
  const bool speed_valid =
      !dashboard_can_lost &&
      (values.status_flags & STATUS_VEHICLE_VALID) != 0U;
  int32_t speed_centi_mps = values.speed_centi_mps;
  if (speed_centi_mps < 0) speed_centi_mps = -speed_centi_mps;
  if (speed_valid && speed_centi_mps != 0)
  {
    dino_distance_px +=
        (speed_centi_mps / 100.0f) * DINO_PIXELS_PER_M *
        (elapsed_ms / 1000.0f);
    if (dino_jump_delay_ms != 0U)
    {
      if (elapsed_ms >= dino_jump_delay_ms)
      {
        const uint32_t overshoot_ms = elapsed_ms - dino_jump_delay_ms;
        dino_jump_start_px = dino_distance_px -
            (speed_centi_mps / 100.0f) * DINO_PIXELS_PER_M *
            (overshoot_ms / 1000.0f);
        dino_jump_delay_ms = 0U;
      }
      else
      {
        dino_jump_delay_ms -= elapsed_ms;
      }
    }
  }

  const float gap = (float)dinoObstacleX() - DINO_RIGHT_X;
  if (dino_jump_start_px < 0.0f &&
      gap >= 0.0f && gap <= DINO_JUMP_TRIGGER_GAP_PX)
  {
    dino_jump_start_px = dino_distance_px;
    dino_jump_delay_ms = DINO_JUMP_DELAY_MS;
  }
  if (dino_jump_start_px >= 0.0f && dino_jump_delay_ms == 0U &&
      dino_distance_px - dino_jump_start_px >= DINO_JUMP_DISTANCE_PX)
  {
    dino_jump_start_px = -1.0f;
  }
}

void fillDinoBlock(uint16_t y, int16_t x, int16_t top, Rgb color)
{
  if ((int16_t)y < top || (int16_t)y > top + 1 ||
      x > (int16_t)DINO_REGION_X1 || x + 1 < (int16_t)DINO_REGION_X0)
  {
    return;
  }
  const int16_t clipped_x0 = x < (int16_t)DINO_REGION_X0
                                 ? (int16_t)DINO_REGION_X0 : x;
  const int16_t clipped_x1 = x + 1 > (int16_t)DINO_REGION_X1
                                 ? (int16_t)DINO_REGION_X1 : x + 1;
  fillRowRect(y, (uint16_t)clipped_x0, (uint16_t)top,
             (uint16_t)clipped_x1, (uint16_t)(top + 1), color);
}

void drawDinoRow(uint16_t y, const ManualFrameData &frame)
{
  if (!frame.dino_visible) return;
  fillRowRect(y, DINO_REGION_X0, DINO_GROUND_Y,
             DINO_REGION_X1, DINO_GROUND_Y, COLOR_DINO_GROUND);
  for (int16_t x =
           (int16_t)(285 - frame.dino_ground_scroll);
       x <= (int16_t)DINO_REGION_X1;
       x += 24)
  {
    const int16_t x0 = x < (int16_t)DINO_REGION_X0
                           ? (int16_t)DINO_REGION_X0 : x;
    const int16_t x1 = x + 3 > (int16_t)DINO_REGION_X1
                           ? (int16_t)DINO_REGION_X1 : x + 3;
    if (x1 >= (int16_t)DINO_REGION_X0)
      fillRowRect(y, (uint16_t)x0, DINO_GROUND_Y - 2U,
                  (uint16_t)x1, DINO_GROUND_Y - 2U, COLOR_DINO_GROUND);
  }

  const int16_t dino_top =
      (int16_t)(DINO_GROUND_Y - 22U - frame.dino_jump_px);
  const int16_t sprite_row = (int16_t)y - dino_top;
  if (sprite_row >= 0 && sprite_row < 22)
  {
    const uint32_t bits = DINO_RUN_FRAMES[frame.dino_foot_frame][sprite_row];
    const uint16_t color = toRgb565(COLOR_DINO);
    for (uint8_t column = 0U; column < 24U; ++column)
    {
      if ((bits & (1UL << column)) != 0U)
        row_pixels[DINO_X + column] = color;
    }
  }

  const int16_t cactus_top = (int16_t)(DINO_GROUND_Y - 16U);
  for (uint8_t row = 0U; row < 8U; ++row)
  {
    const int16_t top = (int16_t)(cactus_top + row * 2U);
    for (uint8_t column = 0U; column < 3U; ++column)
    {
      if ((DINO_CACTUS[row] & (1U << column)) != 0U)
        fillDinoBlock(y,
                      (int16_t)(frame.dino_obstacle_x + column * 2U),
                      top,
                      COLOR_DINO);
    }
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

bool trackMembership(uint8_t progress,
                     uint8_t start,
                     uint8_t end,
                     bool valid);

void applyTrack(uint16_t y,
                const ManualTftTrackRow *rows,
                const uint8_t *data,
                uint8_t start,
                uint8_t end,
                Rgb color,
                bool valid)
{
  const ManualTftTrackRow row = rows[y];
  if (row.length == 0U) return;
  for (uint16_t index = 0U; index < row.length; ++index)
  {
    const uint8_t alpha = data[row.offset + index * 2U];
    const uint8_t progress = data[row.offset + index * 2U + 1U];
    if (alpha != 0U && trackMembership(progress, start, end, valid))
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
  const uint32_t timing_started_us = DashboardTiming::beginMeasure();
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
  frame->dino_obstacle_x = dinoObstacleX();
  frame->dino_jump_px = dinoJumpPx();
  frame->dino_foot_frame = (uint8_t)((uint32_t)(dino_distance_px / 8.0f) & 1U);
  frame->dino_ground_scroll = (uint8_t)((uint32_t)dino_distance_px % 24U);
  frame->dino_visible = frame->alert == nullptr;
  DashboardTiming::statePrepared(
      timing_started_us, DashboardTiming::Page::Manual);
}

void composeRow(uint16_t y, const ManualFrameData &frame)
{
  const uint32_t timing_started_us = DashboardTiming::beginMeasure();
  static const int16_t gauge_centers[] = {385, 411, 437, 463};
  static const int16_t status_centers[] = {99, 174, 244, 312, 377, 439};

  decodeStaticRow(y);
  applyTrack(y, manual_track_bus_rows, manual_track_bus_data,
             65U, frame.bus_progress, frame.bus_color, frame.part1_valid);
  applyTrack(y, manual_track_phase_rows, manual_track_phase_data,
             0U, frame.phase_progress, frame.phase_color, frame.part1_valid);
  applyGaugeRows(y, frame);
  drawDinoRow(y, frame);
  if (frame.alert != nullptr)
    fillRowRect(y, 281U, 218U, 470U, 242U, frame.alert_color);

  drawTextRow(y, 35, 48, frame.gear, manual_font_gear,
              TftHorizontalAnchor::Center, COLOR_GEAR);
  const TftFontAsset &speed_font =
      strlen(frame.speed) <= 4U ? manual_font_speed64 : manual_font_speed57;
  drawTextRow(y, 361, 162, frame.speed, speed_font,
              TftHorizontalAnchor::Right, COLOR_BLACK);
  drawTextRow(y, 126, 229, frame.bus, manual_font_current,
              TftHorizontalAnchor::Center,
              frame.part1_valid ? COLOR_BLACK : COLOR_MUTED);
  drawTextRow(y, 197, 229, frame.phase, manual_font_current,
              TftHorizontalAnchor::Center,
              frame.part1_valid ? COLOR_BLACK : COLOR_MUTED);
  for (uint8_t index = 0U; index < 4U; ++index)
  {
    drawTextRow(y, gauge_centers[index], 207, frame.gauge_text[index],
                manual_font_gauge, TftHorizontalAnchor::Center,
                frame.gauge_text_colors[index]);
  }
  for (uint8_t index = 0U; index < 6U; ++index)
  {
    drawTextRow(y, status_centers[index], 295, frame.status_values[index],
                manual_font_status, TftHorizontalAnchor::Center,
                frame.status_colors[index]);
  }
  if (frame.alert != nullptr)
  {
    drawTextRow(y, 375, 230, frame.alert, manual_font_alert,
                TftHorizontalAnchor::Center, COLOR_WHITE);
  }
  DashboardTiming::rowComposed(
      timing_started_us, DashboardTiming::Page::Manual, y);
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

bool dinoChanged(const ManualFrameData &current,
                 const ManualFrameData &previous)
{
  if (!current.dino_visible && !previous.dino_visible) return false;
  return current.dino_visible != previous.dino_visible ||
         current.dino_obstacle_x != previous.dino_obstacle_x ||
         current.dino_jump_px != previous.dino_jump_px ||
         current.dino_foot_frame != previous.dino_foot_frame ||
         current.dino_ground_scroll != previous.dino_ground_scroll;
}

void markChangedRegions(const ManualFrameData &current,
                        const ManualFrameData &previous)
{
  if (alertChanged(current, previous))
  {
    pending_regions |= DIRTY_ALERT;
    pending_regions |= DIRTY_DINO;
  }
  if (strcmp(current.speed, previous.speed) != 0)
    pending_regions |= DIRTY_SPEED;
  if (strcmp(current.bus, previous.bus) != 0 ||
      strcmp(current.phase, previous.phase) != 0 ||
      current.part1_valid != previous.part1_valid)
    pending_regions |= DIRTY_CURRENT;

  static const uint16_t gauge_bits[] = {
      DIRTY_PEDAL, DIRTY_HV, DIRTY_MOTOR, DIRTY_CONTROLLER};
  for (uint8_t index = 0U; index < 4U; ++index)
  {
    if (strcmp(current.gauge_text[index], previous.gauge_text[index]) != 0 ||
        current.gauge_levels[index] != previous.gauge_levels[index] ||
        !sameColor(current.gauge_colors[index], previous.gauge_colors[index]) ||
        !sameColor(current.gauge_text_colors[index],
                   previous.gauge_text_colors[index]))
      pending_regions |= gauge_bits[index];
  }

  if (strcmp(current.gear, previous.gear) != 0)
    pending_regions |= DIRTY_GEAR;
  if (statusChanged(current, previous)) pending_regions |= DIRTY_STATUS;
  if (current.part1_valid != previous.part1_valid ||
      current.bus_progress != previous.bus_progress ||
      current.phase_progress != previous.phase_progress ||
      !sameColor(current.bus_color, previous.bus_color) ||
      !sameColor(current.phase_color, previous.phase_color))
    pending_regions |= DIRTY_TRACK;
  if (dinoChanged(current, previous)) pending_regions |= DIRTY_DINO;
}

void regionForBit(uint16_t bit,
                  uint16_t *x0,
                  uint16_t *y0,
                  uint16_t *x1,
                  uint16_t *y1)
{
  if (bit == DIRTY_ALERT) {*x0 = 281U; *y0 = 218U; *x1 = 470U; *y1 = 242U;}
  else if (bit == DIRTY_SPEED) {*x0 = 185U; *y0 = 130U; *x1 = 367U; *y1 = 193U;}
  else if (bit == DIRTY_CURRENT) {*x0 = 88U; *y0 = 214U; *x1 = 238U; *y1 = 244U;}
  else if (bit == DIRTY_PEDAL) {*x0 = 371U; *y0 = 92U; *x1 = 400U; *y1 = 226U;}
  else if (bit == DIRTY_HV) {*x0 = 397U; *y0 = 92U; *x1 = 426U; *y1 = 226U;}
  else if (bit == DIRTY_MOTOR) {*x0 = 423U; *y0 = 92U; *x1 = 452U; *y1 = 226U;}
  else if (bit == DIRTY_CONTROLLER) {*x0 = 449U; *y0 = 92U; *x1 = 479U; *y1 = 226U;}
  else if (bit == DIRTY_GEAR) {*x0 = 4U; *y0 = 12U; *x1 = 67U; *y1 = 82U;}
  else if (bit == DIRTY_STATUS) {*x0 = 62U; *y0 = 284U; *x1 = 471U; *y1 = 306U;}
  else {*x0 = DINO_REGION_X0; *y0 = DINO_REGION_Y0;
        *x1 = DINO_REGION_X1; *y1 = DINO_REGION_Y1;}
}

uint16_t selectRoundRobin(uint16_t regions)
{
  for (uint8_t offset = 0U; offset < 11U; ++offset)
  {
    const uint8_t index = (uint8_t)((round_robin_index + offset) % 11U);
    const uint16_t bit = (uint16_t)(1U << index);
    if ((regions & bit) != 0U)
    {
      round_robin_index = (uint8_t)((index + 1U) % 11U);
      return bit;
    }
  }
  return 0U;
}

bool trackMembership(uint8_t progress,
                     uint8_t start,
                     uint8_t end,
                     bool valid)
{
  if (!valid || start == end) return false;
  const uint8_t low = start < end ? start : end;
  const uint8_t high = start < end ? end : start;
  return progress >= low && progress <= high;
}

bool trackChangedSpan(uint16_t y,
                      const ManualFrameData &frame,
                      uint16_t *x0,
                      uint16_t *x1)
{
  bool found = false;
  const ManualTftTrackRow rows[2] = {
      manual_track_bus_rows[y], manual_track_phase_rows[y]};
  const uint8_t previous[2] = {rendered_bus_progress, rendered_phase_progress};
  const uint8_t current[2] = {frame.bus_progress, frame.phase_progress};
  const uint8_t start[2] = {65U, 0U};
  const uint8_t *data[2] = {manual_track_bus_data, manual_track_phase_data};
  for (uint8_t track = 0U; track < 2U; ++track)
  {
    const ManualTftTrackRow &row = rows[track];
    for (uint16_t index = 0U; index < row.length; ++index)
    {
      const uint8_t alpha = data[track][row.offset + index * 2U];
      const uint8_t progress = data[track][row.offset + index * 2U + 1U];
      const bool changed = active_track_full ||
          trackMembership(progress, start[track], previous[track],
                          rendered_part1_valid) !=
          trackMembership(progress, start[track], current[track],
                          frame.part1_valid);
      if (alpha == 0U || !changed) continue;
      const uint16_t x = (uint16_t)(row.x + index);
      if (!found)
      {
        *x0 = x;
        *x1 = x;
        found = true;
      }
      else
      {
        if (x < *x0) *x0 = x;
        if (x > *x1) *x1 = x;
      }
    }
  }
  return found;
}

float smoothstep(float value)
{
  if (value < 0.0f) value = 0.0f;
  if (value > 1.0f) value = 1.0f;
  return value * value * (3.0f - 2.0f * value);
}

void blendStartupRuns(uint16_t *pixels,
                      uint16_t length,
                      const uint8_t *data,
                      uint16_t data_length,
                      Rgb color,
                      float alpha_scale,
                      int16_t row_x,
                      float frontier_x,
                      bool draw_frontier)
{
  uint16_t position = 0U;
  for (uint16_t offset = 0U; offset + 1U < data_length; offset += 2U)
  {
    const uint8_t count = data[offset];
    const uint8_t value = data[offset + 1U];
    if (value != 0U)
    {
      uint8_t alpha = (uint8_t)lroundf(value * alpha_scale);
      for (uint16_t index = 0U; index < count && position + index < length; ++index)
      {
        const int16_t x = (int16_t)(row_x + position + index);
        pixels[position + index] = blend(pixels[position + index], color, alpha);
        if (draw_frontier && fabsf((float)x - frontier_x) <= 10.0f)
        {
          const uint8_t frontier_alpha =
              (uint8_t)(((uint16_t)value * 150U + 127U) / 255U);
          pixels[position + index] =
              blend(pixels[position + index], COLOR_WHITE, frontier_alpha);
        }
      }
    }
    position = (uint16_t)(position + count);
  }
}

bool renderStartupFrame(uint16_t elapsed_ms,
                        uint32_t deadline_started_ms,
                        bool enforce_deadline)
{
  DashboardTiming::startupFrameEvent(false, elapsed_ms, micros());
  const uint32_t state_started_us = DashboardTiming::beginMeasure();
  float outline_scale = 1.0f;
  float haze_strength = 0.02f;
  float reveal_boundary = 1000.0f;
  bool reveal_active = false;
  if (elapsed_ms < STARTUP_REVEAL_END_MS)
  {
    const float progress = smoothstep(
        elapsed_ms / (float)STARTUP_REVEAL_END_MS);
    reveal_boundary = 60.0f + progress * 370.0f;
    haze_strength = 0.12f;
    reveal_active = true;
  }
  else if (elapsed_ms < STARTUP_HOLD_END_MS)
  {
    const float progress =
        (elapsed_ms - STARTUP_REVEAL_END_MS) /
        (float)(STARTUP_HOLD_END_MS - STARTUP_REVEAL_END_MS);
    haze_strength = 0.12f * (1.0f - smoothstep(progress)) + 0.02f;
  }
  else
  {
    const float fade = 1.0f - smoothstep(
        (elapsed_ms - STARTUP_HOLD_END_MS) /
        (float)(STARTUP_DURATION_MS - STARTUP_HOLD_END_MS));
    outline_scale = fade;
    haze_strength = 0.02f * fade;
  }
  DashboardTiming::statePrepared(
      state_started_us, DashboardTiming::Page::Startup);

  for (uint16_t y = STARTUP_BBOX_Y0; y <= STARTUP_BBOX_Y1; ++y)
  {
    const uint16_t row_index = (uint16_t)(y - STARTUP_BBOX_Y0);
    const StartupTftMaskRow &row = startup_mask_rows[row_index];
    if (row.length == 0U) continue;
    const uint32_t row_started_us = DashboardTiming::beginMeasure();
    const StartupTftMaskRow &next = startup_mask_rows[row_index + 1U];
    for (uint16_t index = 0U; index < row.length; ++index)
      row_pixels[index] = St7796Lcd::COLOR_BLACK;

    const float frontier_x = reveal_boundary + 20.0f -
        40.0f * y / (float)St7796Lcd::HEIGHT;
    uint16_t visible_length = row.length;
    if (reveal_active)
    {
      if (frontier_x < row.x) visible_length = 0U;
      else if (frontier_x < row.x + row.length)
        visible_length = (uint16_t)(frontier_x - row.x + 1.0f);
    }

    if (visible_length != 0U)
    {
      const float haze_scale =
          (STARTUP_HAZE_MAXIMUM / 15.0f) * haze_strength;
      blendStartupRuns(
          row_pixels, visible_length,
          startup_haze_data + row.haze_offset,
          (uint16_t)(next.haze_offset - row.haze_offset),
          {214U, 214U, 214U}, haze_scale,
          (int16_t)row.x, frontier_x, false);
      blendStartupRuns(
          row_pixels, visible_length,
          startup_outline_data + row.outline_offset,
          (uint16_t)(next.outline_offset - row.outline_offset),
          {242U, 242U, 242U}, outline_scale,
          (int16_t)row.x, frontier_x, reveal_active);
    }
    DashboardTiming::rowComposed(
        row_started_us, DashboardTiming::Page::Startup, y);
    St7796Lcd::beginPixelStream(
        row.x, y, (uint16_t)(row.x + row.length - 1U), y);
    St7796Lcd::writePixelRow(row_pixels, row.length);
    St7796Lcd::endPixelStream();
    if (enforce_deadline &&
        (uint32_t)(millis() - deadline_started_ms) >= STARTUP_DURATION_MS)
    {
      DashboardTiming::startupFrameEvent(true, elapsed_ms, micros());
      return false;
    }
  }
  DashboardTiming::startupFrameEvent(true, elapsed_ms, micros());
  return true;
}
}  // namespace

namespace ManualTftUi
{

void playStartup()
{
  DashboardTiming::startupEvent(false, micros());
  St7796Lcd::fillScreen(St7796Lcd::COLOR_BLACK);
  const uint32_t started_ms = millis();
  while ((uint32_t)(millis() - started_ms) < STARTUP_DURATION_MS)
  {
    const uint16_t elapsed_ms =
        (uint16_t)(millis() - started_ms);
    if (!renderStartupFrame(elapsed_ms, started_ms, true)) break;
  }
  St7796Lcd::fillRect(
      STARTUP_BBOX_X0, STARTUP_BBOX_Y0,
      (uint16_t)(STARTUP_BBOX_X1 - STARTUP_BBOX_X0 + 1U),
      (uint16_t)(STARTUP_BBOX_Y1 - STARTUP_BBOX_Y0 + 1U),
      St7796Lcd::COLOR_BLACK);
  DashboardTiming::startupEvent(true, micros());
}

#if defined(DASHBOARD_BENCH_INPUT) && \
    defined(DASHBOARD_TIMING_STARTUP_PROFILE)
void renderStartupFrameForTiming(uint16_t elapsed_ms)
{
  (void)renderStartupFrame(elapsed_ms, 0U, false);
}
#endif

void observe(const DashboardValues &values,
             bool dashboard_can_lost,
             uint32_t now_ms)
{
  advanceDino(values, dashboard_can_lost, now_ms);
}

void begin(const DashboardValues &values, bool dashboard_can_lost)
{
  buildFrameData(values, dashboard_can_lost, &desired_frame);
  renderFullFrame(desired_frame);
  previous_queued_frame = desired_frame;
  have_queued_frame = true;
  pending_regions = 0U;
  urgent_regions = 0U;
  active_region = 0U;
  rendered_track_valid = true;
  rendered_bus_progress = desired_frame.bus_progress;
  rendered_phase_progress = desired_frame.phase_progress;
  rendered_bus_color = desired_frame.bus_color;
  rendered_phase_color = desired_frame.phase_color;
  rendered_part1_valid = desired_frame.part1_valid;
}

void renderPreparedRows(uint16_t y0, uint16_t y1)
{
  renderRegion(desired_frame, 0U, y0, St7796Lcd::WIDTH - 1U, y1);
}

void synchronize()
{
  previous_queued_frame = desired_frame;
  have_queued_frame = true;
  pending_regions |= 0x07FFU;
  urgent_regions = 0U;
  active_region = 0U;
  rendered_track_valid = false;
}

void resetState()
{
  have_queued_frame = false;
  pending_regions = 0U;
  urgent_regions = 0U;
  active_region = 0U;
  active_y = 0U;
  round_robin_index = 0U;
  rendered_track_valid = false;
  dino_distance_px = 0.0f;
  dino_jump_start_px = -1.0f;
  dino_jump_delay_ms = 0U;
  dino_last_update_ms = 0U;
  dino_time_initialized = false;
}

void update(const DashboardValues &values,
            bool dashboard_can_lost,
            bool urgent_change)
{
  ManualFrameData current = {};
  buildFrameData(values, dashboard_can_lost, &current);
  if (!have_queued_frame)
  {
    desired_frame = current;
    previous_queued_frame = current;
    have_queued_frame = true;
    pending_regions = 0x07FFU;
    return;
  }
  markChangedRegions(current, previous_queued_frame);
  desired_frame = current;
  previous_queued_frame = current;
  if (urgent_change)
  {
    if (active_region != 0U)
    {
      pending_regions |= active_region;
      if (active_region == DIRTY_TRACK) rendered_track_valid = false;
    }
    active_region = 0U;
    urgent_regions |= pending_regions & URGENT_REGIONS;
  }
}

void service(uint32_t now_ms)
{
  (void)now_ms;
  const uint32_t service_started_us = micros();
  if (active_region == 0U)
  {
    if (pending_regions == 0U) return;
    const uint16_t candidates = urgent_regions != 0U
                                    ? urgent_regions : pending_regions;
    active_region = selectRoundRobin(candidates);
    pending_regions &= (uint16_t)~active_region;
    urgent_regions &= (uint16_t)~active_region;
    active_frame = desired_frame;
    active_y = active_region == DIRTY_TRACK ? 0U : DINO_REGION_Y0;
    if (active_region == DIRTY_TRACK)
    {
      active_track_full = !rendered_track_valid ||
          active_frame.part1_valid != rendered_part1_valid ||
          !sameColor(active_frame.bus_color, rendered_bus_color) ||
          !sameColor(active_frame.phase_color, rendered_phase_color);
    }
    if (active_region != DIRTY_TRACK)
    {
      uint16_t x0, y0, x1, y1;
      regionForBit(active_region, &x0, &y0, &x1, &y1);
      active_y = y0;
    }
  }

  if (active_region == DIRTY_TRACK)
  {
    uint8_t rendered = 0U;
    while (active_y < St7796Lcd::HEIGHT && rendered < RENDER_ROWS_PER_SERVICE)
    {
      uint16_t x0 = 0U;
      uint16_t x1 = 0U;
      if (trackChangedSpan(active_y, active_frame, &x0, &x1))
      {
        renderRegion(active_frame, x0, active_y, x1, active_y);
        rendered++;
        active_y++;
        if ((uint32_t)(micros() - service_started_us) >= 2000U) break;
        continue;
      }
      active_y++;
      if ((uint32_t)(micros() - service_started_us) >= 2000U) break;
    }
    if (active_y >= St7796Lcd::HEIGHT)
    {
      rendered_bus_progress = active_frame.bus_progress;
      rendered_phase_progress = active_frame.phase_progress;
      rendered_bus_color = active_frame.bus_color;
      rendered_phase_color = active_frame.phase_color;
      rendered_part1_valid = active_frame.part1_valid;
      rendered_track_valid = true;
      DashboardTiming::regionCompleted(
          DashboardTiming::Page::Manual, active_region, millis());
      active_region = 0U;
    }
    return;
  }

  uint16_t x0, y0, x1, y1;
  regionForBit(active_region, &x0, &y0, &x1, &y1);
  uint8_t rendered = 0U;
  while (active_y <= y1 && rendered < RENDER_ROWS_PER_SERVICE)
  {
    renderRegion(active_frame, x0, active_y, x1, active_y);
    active_y++;
    rendered++;
    if ((uint32_t)(micros() - service_started_us) >= 2000U) break;
  }
  if (active_y > y1)
  {
    DashboardTiming::regionCompleted(
        DashboardTiming::Page::Manual, active_region, millis());
    active_region = 0U;
  }
}

}  // namespace ManualTftUi
