#include "include/auto_tft_ui.h"

#include <Arduino.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "assets/auto_tft_assets.h"
#include "assets/auto_tft_dynamic_assets.h"
#include "include/dashboard_timing.h"
#include "include/st7796_lcd.h"

namespace
{

constexpr uint16_t rgb565(uint8_t red, uint8_t green, uint8_t blue)
{
  return (uint16_t)(((uint16_t)(red >> 3) << 11) |
                    ((uint16_t)(green >> 2) << 5) |
                    (blue >> 3));
}

static constexpr uint16_t WHITE = rgb565(255U, 255U, 255U);
static constexpr uint16_t INK = rgb565(19U, 28U, 37U);
static constexpr uint16_t MUTED = rgb565(104U, 119U, 131U);
static constexpr uint16_t BLUE = rgb565(37U, 134U, 229U);
static constexpr uint16_t VIOLET = rgb565(152U, 101U, 217U);
static constexpr uint16_t GREEN = rgb565(37U, 173U, 105U);
static constexpr uint16_t YELLOW = rgb565(255U, 212U, 59U);
static constexpr uint16_t ORANGE = rgb565(243U, 138U, 44U);
static constexpr uint16_t RED = rgb565(233U, 72U, 72U);
static constexpr uint16_t HOLD = rgb565(148U, 191U, 229U);
static constexpr uint16_t OFF = rgb565(38U, 54U, 66U);
static constexpr uint16_t UNKNOWN = rgb565(100U, 117U, 130U);
static constexpr uint16_t DARK_TEXT = rgb565(185U, 199U, 210U);

static constexpr uint16_t DIRTY_MESSAGE = 1U << 8;
static constexpr uint16_t DIRTY_SPEED = 1U << 9;
static constexpr uint16_t DIRTY_TARGET_SPEED = 1U << 10;
static constexpr uint16_t DIRTY_COMPASS = 1U << 11;
static constexpr uint16_t DIRTY_STEERING = 1U << 12;
static constexpr uint16_t DIRTY_STEER_TEXT = 1U << 13;
static constexpr uint16_t DIRTY_TRACE = 1U << 14;
static constexpr uint16_t DIRTY_ALL = 0x7FFFU;
static constexpr uint16_t URGENT_REGIONS =
    0x00FFU | DIRTY_MESSAGE | DIRTY_STEER_TEXT;
static constexpr uint8_t RENDER_ROWS_PER_SERVICE = 4U;
static constexpr uint32_t TRACE_REDRAW_PERIOD_MS = 200U;
static constexpr size_t CURRENT_HISTORY_SIZE = 128U;
static constexpr float kPi = 3.14159265358979323846f;
static constexpr int32_t KEYA_COUNTS_PER_REV = 10000;

struct Badge
{
  char label[11];
  char state[14];
  uint16_t background;
  uint16_t foreground;
};

struct ArrowGeometry
{
  float x0;
  float y0;
  float x1;
  float y1;
  float head_x[3];
  float head_y[3];
  uint16_t color;
  uint8_t thickness;
  uint8_t valid;
};

struct TraceGeometry
{
  float x;
  float y;
  uint8_t valid;
};

struct AutoFrame
{
  Badge badges[8];
  char message_line1[25];
  char message_line2[25];
  uint16_t message_color;
  char speed[8];
  char target_speed[8];
  char heading[7];
  char target_heading[7];
  char actual_angle[17];
  char target_angle[17];
  char current[10];
  char steer_state[15];
  char output[5];
  char feedback[7];
  uint16_t steer_state_color;
  uint16_t feedback_color;
  uint16_t heading_centideg;
  uint16_t target_heading_centideg;
  int32_t actual_motor_count;
  int32_t target_motor_count;
  uint8_t heading_valid;
  uint8_t target_heading_valid;
  uint8_t actual_motor_valid;
  uint8_t target_motor_valid;
  ArrowGeometry target_heading_arrow;
  ArrowGeometry actual_steering_arrow;
  ArrowGeometry target_steering_arrow;
  float vehicle_x[5];
  float vehicle_y[5];
  float window_x[4];
  float window_y[4];
  float window_line_x[2];
  float window_line_y[2];
  TraceGeometry trace[CURRENT_HISTORY_SIZE];
  uint8_t trace_count;
  char trace_peak[17];
};

struct CurrentPoint
{
  uint32_t time_ms;
  int16_t deciamp;
  uint8_t valid;
};

struct Region
{
  uint16_t x0;
  uint16_t y0;
  uint16_t x1;
  uint16_t y1;
};

static uint16_t row_pixels[St7796Lcd::WIDTH];
static AutoFrame desired_frame = {};
static AutoFrame previous_queued_frame = {};
static AutoFrame active_frame = {};
static bool have_queued_frame = false;
static uint16_t pending_regions = 0U;
static uint16_t urgent_regions = 0U;
static uint16_t active_region = 0U;
static uint16_t active_y = 0U;
static uint8_t round_robin_index = 0U;
static uint32_t compose_now_ms = 0U;
static uint32_t last_trace_redraw_ms = 0U;
static CurrentPoint current_history[CURRENT_HISTORY_SIZE] = {};
static size_t current_history_head = 0U;
static size_t current_history_count = 0U;

void decodeStaticRow(uint16_t y)
{
  const uint32_t begin = auto_static_row_offsets[y];
  const uint32_t end = auto_static_row_offsets[y + 1U];
  uint16_t x = 0U;
  for (uint32_t offset = begin; offset + 2U < end; offset += 3U)
  {
    const uint8_t count = auto_static_row_rle[offset];
    const uint16_t color =
        ((uint16_t)auto_static_row_rle[offset + 1U] << 8) |
        auto_static_row_rle[offset + 2U];
    for (uint8_t index = 0U; index < count; ++index) row_pixels[x++] = color;
  }
}

void fillRow(uint16_t y,
             int16_t x0,
             int16_t y0,
             int16_t x1,
             int16_t y1,
             uint16_t color)
{
  if ((int16_t)y < y0 || (int16_t)y > y1) return;
  if (x0 < 0) x0 = 0;
  if (x1 >= (int16_t)St7796Lcd::WIDTH) x1 = St7796Lcd::WIDTH - 1;
  for (int16_t x = x0; x <= x1; ++x) row_pixels[x] = color;
}

void drawTextRow(uint16_t y,
                 int16_t anchor_x,
                 int16_t anchor_y,
                 const char *text,
                 const TftFontAsset &font,
                 uint16_t color,
                 TftHorizontalAnchor anchor)
{
  TftFontRenderer::drawTextRow(
      row_pixels, St7796Lcd::WIDTH, y, anchor_x, anchor_y,
      text, font, color, anchor);
}

void drawLineRow(uint16_t y,
                 float x0,
                 float y0,
                 float x1,
                 float y1,
                 uint16_t color,
                 uint8_t thickness = 1U)
{
  const float minimum_y = fminf(y0, y1) - thickness;
  const float maximum_y = fmaxf(y0, y1) + thickness;
  if ((float)y < minimum_y || (float)y > maximum_y) return;
  if (fabsf(y1 - y0) < 0.01f)
  {
    if (fabsf((float)y - y0) <= thickness)
      fillRow(y, (int16_t)lroundf(fminf(x0, x1)), y,
              (int16_t)lroundf(fmaxf(x0, x1)), y, color);
    return;
  }
  const float ratio = ((float)y - y0) / (y1 - y0);
  if (ratio < -0.05f || ratio > 1.05f) return;
  const int16_t x = (int16_t)lroundf(x0 + ratio * (x1 - x0));
  fillRow(y, x - thickness, y, x + thickness, y, color);
}

void fillPolygonRow(uint16_t y,
                    const float *xs,
                    const float *ys,
                    uint8_t count,
                    uint16_t color)
{
  float intersections[8];
  uint8_t found = 0U;
  const float scan_y = (float)y + 0.5f;
  for (uint8_t index = 0U; index < count; ++index)
  {
    const uint8_t next = (uint8_t)((index + 1U) % count);
    if ((ys[index] <= scan_y && ys[next] > scan_y) ||
        (ys[next] <= scan_y && ys[index] > scan_y))
    {
      intersections[found++] = xs[index] +
          (scan_y - ys[index]) * (xs[next] - xs[index]) /
          (ys[next] - ys[index]);
    }
  }
  for (uint8_t left = 0U; left + 1U < found; ++left)
  {
    for (uint8_t right = left + 1U; right < found; ++right)
    {
      if (intersections[right] < intersections[left])
      {
        const float value = intersections[left];
        intersections[left] = intersections[right];
        intersections[right] = value;
      }
    }
  }
  for (uint8_t index = 0U; index + 1U < found; index += 2U)
  {
    fillRow(y,
            (int16_t)ceilf(intersections[index]), y,
            (int16_t)floorf(intersections[index + 1U]), y,
            color);
  }
}

void prepareArrow(ArrowGeometry *arrow,
                  float cx,
                  float cy,
                  float angle_deg,
                  float length,
                  uint16_t color,
                  uint8_t thickness,
                  bool valid)
{
  memset(arrow, 0, sizeof(*arrow));
  arrow->valid = valid ? 1U : 0U;
  if (!valid) return;
  const float radians = angle_deg * kPi / 180.0f;
  const float dx = sinf(radians);
  const float dy = -cosf(radians);
  arrow->x0 = cx;
  arrow->y0 = cy;
  arrow->x1 = cx + dx * length;
  arrow->y1 = cy + dy * length;
  arrow->head_x[0] = arrow->x1;
  arrow->head_x[1] = arrow->x1 - dx * 10.0f - dy * 5.0f;
  arrow->head_x[2] = arrow->x1 - dx * 10.0f + dy * 5.0f;
  arrow->head_y[0] = arrow->y1;
  arrow->head_y[1] = arrow->y1 - dy * 10.0f + dx * 5.0f;
  arrow->head_y[2] = arrow->y1 - dy * 10.0f - dx * 5.0f;
  arrow->color = color;
  arrow->thickness = thickness;
}

void drawArrowRow(uint16_t y, const ArrowGeometry &arrow)
{
  if (arrow.valid == 0U) return;
  drawLineRow(y, arrow.x0, arrow.y0, arrow.x1, arrow.y1,
              arrow.color, arrow.thickness);
  fillPolygonRow(y, arrow.head_x, arrow.head_y, 3U, arrow.color);
}

void drawCompassRow(uint16_t y, const AutoFrame &frame)
{
  drawArrowRow(y, frame.target_heading_arrow);
  if (frame.heading_valid == 0U)
  {
    drawTextRow(y, 80, 145, "--", auto_font_reading, MUTED,
                TftHorizontalAnchor::Center);
    return;
  }
  fillPolygonRow(y, frame.vehicle_x, frame.vehicle_y, 5U, BLUE);
  fillPolygonRow(y, frame.window_x, frame.window_y, 4U, WHITE);
  drawLineRow(y, frame.window_line_x[0], frame.window_line_y[0],
              frame.window_line_x[1], frame.window_line_y[1], WHITE, 1U);
}

float motorAngleDeg(int32_t count)
{
  return (float)count * 360.0f / (float)KEYA_COUNTS_PER_REV;
}

void prepareFrameGeometry(AutoFrame *frame, uint32_t now_ms)
{
  prepareArrow(&frame->target_heading_arrow, 80.0f, 145.0f,
               frame->target_heading_centideg / 100.0f,
               44.0f, VIOLET, 1U, frame->target_heading_valid != 0U);
  prepareArrow(&frame->target_steering_arrow, 400.0f, 145.0f,
               -fmodf(motorAngleDeg(frame->target_motor_count), 360.0f),
               48.0f, VIOLET, 2U, frame->target_motor_valid != 0U);
  prepareArrow(&frame->actual_steering_arrow, 400.0f, 145.0f,
               -fmodf(motorAngleDeg(frame->actual_motor_count), 360.0f),
               39.0f, BLUE, 2U, frame->actual_motor_valid != 0U);

  if (frame->heading_valid != 0U)
  {
    const float radians =
        (frame->heading_centideg / 100.0f) * kPi / 180.0f;
    const float cosine = cosf(radians);
    const float sine = sinf(radians);
    static const float body_x[5] = {0.0f, 10.0f, 10.0f, -10.0f, -10.0f};
    static const float body_y[5] = {-24.0f, -13.0f, 17.0f, 17.0f, -13.0f};
    static const float window_x[4] = {-6.0f, 6.0f, 6.0f, -6.0f};
    static const float window_y[4] = {-8.0f, -8.0f, -2.0f, -2.0f};
    for (uint8_t index = 0U; index < 5U; ++index)
    {
      frame->vehicle_x[index] =
          80.0f + body_x[index] * cosine - body_y[index] * sine;
      frame->vehicle_y[index] =
          145.0f + body_x[index] * sine + body_y[index] * cosine;
    }
    for (uint8_t index = 0U; index < 4U; ++index)
    {
      frame->window_x[index] =
          80.0f + window_x[index] * cosine - window_y[index] * sine;
      frame->window_y[index] =
          145.0f + window_x[index] * sine + window_y[index] * cosine;
    }
    frame->window_line_x[0] = 80.0f - 6.0f * cosine - 10.0f * sine;
    frame->window_line_y[0] = 145.0f - 6.0f * sine + 10.0f * cosine;
    frame->window_line_x[1] = 80.0f + 6.0f * cosine - 10.0f * sine;
    frame->window_line_y[1] = 145.0f + 6.0f * sine + 10.0f * cosine;
  }

  int16_t peak = INT16_MIN;
  frame->trace_count = 0U;
  for (size_t index = 0U; index < current_history_count; ++index)
  {
    const size_t slot =
        (current_history_head + CURRENT_HISTORY_SIZE - current_history_count + index) %
        CURRENT_HISTORY_SIZE;
    const CurrentPoint &point = current_history[slot];
    const uint32_t age = (uint32_t)(now_ms - point.time_ms);
    TraceGeometry &trace = frame->trace[frame->trace_count++];
    trace.valid = age <= 5000U && point.valid != 0U ? 1U : 0U;
    if (trace.valid == 0U) continue;
    if (point.deciamp > peak) peak = point.deciamp;
    trace.x = 470.0f - (float)age * 161.0f / 5000.0f;
    float amps = point.deciamp / 10.0f;
    if (amps < 0.0f) amps = 0.0f;
    if (amps > 25.0f) amps = 25.0f;
    trace.y = 301.0f - amps * 26.0f / 25.0f;
  }
  if (peak == INT16_MIN)
    (void)snprintf(frame->trace_peak, sizeof(frame->trace_peak), "5s PEAK -- A");
  else
    (void)snprintf(frame->trace_peak, sizeof(frame->trace_peak), "5s PEAK %ld A",
                   (long)lroundf(peak / 10.0f));
}

void drawSteeringRow(uint16_t y, const AutoFrame &frame)
{
  drawArrowRow(y, frame.target_steering_arrow);
  drawArrowRow(y, frame.actual_steering_arrow);
}

uint16_t badgeTextColor(uint16_t background)
{
  return (background == OFF || background == UNKNOWN) ? WHITE : INK;
}

void setBadge(Badge *badge,
              const char *label,
              const char *state,
              uint16_t background)
{
  (void)snprintf(badge->label, sizeof(badge->label), "%s", label);
  (void)snprintf(badge->state, sizeof(badge->state), "%s", state);
  badge->background = background;
  badge->foreground = badgeTextColor(background);
}

void buildGpsBadge(uint8_t code, Badge *badge)
{
  switch (code)
  {
    case AUTO_GPS_FIXED: setBadge(badge, "GPS", "FIXED", GREEN); break;
    case AUTO_GPS_FLOAT: setBadge(badge, "GPS", "FLOAT", YELLOW); break;
    case AUTO_GPS_FIX: setBadge(badge, "GPS", "FIX", ORANGE); break;
    case AUTO_GPS_NO_FIX: setBadge(badge, "GPS", "NO FIX", RED); break;
    default: setBadge(badge, "GPS", "WAIT", UNKNOWN); break;
  }
}

void buildStreamBadge(const char *label, uint8_t code, Badge *badge)
{
  if (code == AUTO_STREAM_RX) setBadge(badge, label, "RX", GREEN);
  else if (code == AUTO_STREAM_LOST) setBadge(badge, label, "LOST", RED);
  else setBadge(badge, label, "WAIT", UNKNOWN);
}

void buildDriveBadge(uint8_t code, Badge *badge)
{
  switch (code)
  {
    case AUTO_DRIVE_RUN: setBadge(badge, "DRIVE", "RUN", GREEN); break;
    case AUTO_DRIVE_HOLD: setBadge(badge, "DRIVE", "HOLD", HOLD); break;
    case AUTO_DRIVE_CMD_LOST: setBadge(badge, "DRIVE", "CMD LOST", RED); break;
    case AUTO_DRIVE_NO_FEEDBACK: setBadge(badge, "DRIVE", "NO FEEDBACK", RED); break;
    case AUTO_DRIVE_RUN_MISMATCH: setBadge(badge, "DRIVE", "RUN MISMATCH", RED); break;
    case AUTO_DRIVE_FAULT: setBadge(badge, "DRIVE", "FAULT", RED); break;
    case AUTO_DRIVE_STATUS_LOST: setBadge(badge, "DRIVE", "STATUS LOST", RED); break;
    case AUTO_DRIVE_WAIT_CMD: setBadge(badge, "DRIVE", "WAIT CMD", YELLOW); break;
    default: setBadge(badge, "DRIVE", "WAIT", UNKNOWN); break;
  }
}

const char *steerStateText(uint8_t code)
{
  switch (code)
  {
    case AUTO_STEER_ACTIVE: return "ACTIVE";
    case AUTO_STEER_WAIT_CMD: return "WAIT CMD";
    case AUTO_STEER_STATUS_LOST: return "STATUS LOST";
    case AUTO_STEER_MOTOR_FAULT: return "MOTOR FAULT";
    case AUTO_STEER_CAN_LOST: return "CAN LOST";
    case AUTO_STEER_POS_LOST: return "POS LOST";
    case AUTO_STEER_CMD_LOST: return "CMD LOST";
    case AUTO_STEER_TRACK_FAULT: return "TRACK FAULT";
    case AUTO_STEER_TRACK_WARN: return "TRACK WARN";
    case AUTO_STEER_BRAKE: return "BRAKE";
    case AUTO_STEER_ESTOP: return "E-STOP";
    case AUTO_STEER_DISABLED: return "DISABLED";
    default: return "WAIT";
  }
}

uint16_t steerStateColor(uint8_t code)
{
  if (code == AUTO_STEER_ACTIVE) return GREEN;
  if (code == AUTO_STEER_TRACK_WARN) return YELLOW;
  if (code == AUTO_STEER_WAIT_CMD || code == AUTO_STEER_BRAKE ||
      code == AUTO_STEER_DISABLED) return HOLD;
  if (code >= AUTO_STEER_STATUS_LOST && code <= AUTO_STEER_TRACK_FAULT)
    return RED;
  if (code == AUTO_STEER_ESTOP) return RED;
  return UNKNOWN;
}

void buildSteerBadge(uint8_t code, Badge *badge)
{
  setBadge(badge, "STEER", steerStateText(code), steerStateColor(code));
}

void buildBrakeBadge(uint8_t code, Badge *badge)
{
  switch (code)
  {
    case AUTO_BRAKE_OFF: setBadge(badge, "BRAKE CMD", "OFF", OFF); break;
    case AUTO_BRAKE_ON: setBadge(badge, "BRAKE CMD", "ON", YELLOW); break;
    case AUTO_BRAKE_CMD_LOST: setBadge(badge, "BRAKE CMD", "CMD LOST", RED); break;
    case AUTO_BRAKE_STATUS_LOST: setBadge(badge, "BRAKE CMD", "STATUS LOST", RED); break;
    default: setBadge(badge, "BRAKE CMD", "WAIT", UNKNOWN); break;
  }
}

void setMessage(AutoFrame *frame,
                const char *line1,
                const char *line2,
                uint16_t color)
{
  (void)snprintf(frame->message_line1, sizeof(frame->message_line1), "%s", line1);
  (void)snprintf(frame->message_line2, sizeof(frame->message_line2), "%s", line2);
  frame->message_color = color;
}

bool setErrorMessage(uint8_t steer,
                     uint8_t drive,
                     uint8_t brake,
                     uint8_t lidar,
                     uint8_t camera_left,
                     uint8_t camera_right,
                     uint8_t gps,
                     AutoFrame *frame)
{
  switch (steer)
  {
    case AUTO_STEER_STATUS_LOST: setMessage(frame, "STEERING STATUS", "LOST", RED); return true;
    case AUTO_STEER_MOTOR_FAULT: setMessage(frame, "STEER MOTOR FAULT", "", RED); return true;
    case AUTO_STEER_CAN_LOST: setMessage(frame, "STEER CAN LOST", "", RED); return true;
    case AUTO_STEER_POS_LOST: setMessage(frame, "STEER POSITION", "FEEDBACK LOST", RED); return true;
    case AUTO_STEER_CMD_LOST: setMessage(frame, "STEER COMMAND OR", "MODE LOST", RED); return true;
    case AUTO_STEER_TRACK_FAULT: setMessage(frame, "STEER TRACKING", "FAULT", RED); return true;
    default: break;
  }
  switch (drive)
  {
    case AUTO_DRIVE_CMD_LOST: setMessage(frame, "DRIVE COMMAND LOST", "", RED); return true;
    case AUTO_DRIVE_NO_FEEDBACK: setMessage(frame, "DRIVE MOTOR", "FEEDBACK LOST", RED); return true;
    case AUTO_DRIVE_RUN_MISMATCH: setMessage(frame, "DRIVE RUN FEEDBACK", "MISMATCH", RED); return true;
    case AUTO_DRIVE_FAULT: setMessage(frame, "DRIVE MOTOR FAULT", "", RED); return true;
    case AUTO_DRIVE_STATUS_LOST: setMessage(frame, "DRIVE STATUS LOST", "", RED); return true;
    default: break;
  }
  if (brake == AUTO_BRAKE_CMD_LOST)
  {
    setMessage(frame, "BRAKE COMMAND", "INVALID", RED); return true;
  }
  if (brake == AUTO_BRAKE_STATUS_LOST)
  {
    setMessage(frame, "BRAKE STATUS LOST", "", RED); return true;
  }
  if (lidar == AUTO_STREAM_LOST)
  {
    setMessage(frame, "LIDAR INPUT LOST", "", RED); return true;
  }
  if (camera_left == AUTO_STREAM_LOST)
  {
    setMessage(frame, "CAMERA L INPUT", "LOST", RED); return true;
  }
  if (camera_right == AUTO_STREAM_LOST)
  {
    setMessage(frame, "CAMERA R INPUT", "LOST", RED); return true;
  }
  if (gps == AUTO_GPS_NO_FIX)
  {
    setMessage(frame, "GPS NO FIX", "", RED); return true;
  }
  return false;
}

void buildMessage(const DashboardValues &values,
                  const AutoDashboardValues &auto_values,
                  bool dashboard_can_lost,
                  AutoFrame *frame)
{
  if (dashboard_can_lost)
  {
    setMessage(frame, "DASH CAN LOST", "", RED); return;
  }
  if ((values.status_flags & STATUS_ESTOP_ACTIVE) != 0U)
  {
    setMessage(frame, "E-STOP ACTIVE", "", RED); return;
  }
  if (auto_values.host_valid == 0U)
  {
    setMessage(frame, "AUTO DATA LOST", "", RED); return;
  }
  if (setErrorMessage(auto_values.steer_state,
                      auto_values.drive_state,
                      auto_values.brake_state,
                      auto_values.lidar_state,
                      auto_values.camera_left_state,
                      auto_values.camera_right_state,
                      auto_values.gps_state,
                      frame)) return;
  if (auto_values.heading_centideg == AUTO_HEADING_INVALID)
  {
    setMessage(frame, "VEHICLE HEADING", "UNAVAILABLE", ORANGE); return;
  }
  if (auto_values.target_heading_centideg == AUTO_HEADING_INVALID)
  {
    setMessage(frame, "TARGET HEADING", "UNAVAILABLE", ORANGE); return;
  }
  if (auto_values.steer_state == AUTO_STEER_TRACK_WARN)
  {
    setMessage(frame, "STEER TRACKING", "WARNING", ORANGE); return;
  }
  if (auto_values.steer_state != AUTO_STEER_ACTIVE &&
      auto_values.steer_state != AUTO_STEER_WAIT)
  {
    setMessage(frame, "STEER", steerStateText(auto_values.steer_state), INK); return;
  }
  switch (auto_values.activity)
  {
    case AUTO_ACTIVITY_FOLLOW_ROUTE: setMessage(frame, "FOLLOW ROUTE", "", INK); break;
    case AUTO_ACTIVITY_REJOIN_ROUTE: setMessage(frame, "REJOIN ROUTE", "", INK); break;
    case AUTO_ACTIVITY_TRAFFIC_CONTROL: setMessage(frame, "TRAFFIC CONTROL", "", INK); break;
    case AUTO_ACTIVITY_AVOID_OBSTACLE: setMessage(frame, "AVOID OBSTACLE", "", INK); break;
    case AUTO_ACTIVITY_INTERSECTION: setMessage(frame, "INTERSECTION", "", INK); break;
    case AUTO_ACTIVITY_FINISH: setMessage(frame, "FINISH", "", INK); break;
    case AUTO_ACTIVITY_EMERGENCY: setMessage(frame, "EMERGENCY STOP", "", RED); break;
    case AUTO_ACTIVITY_ACTIVE: setMessage(frame, "AUTO ACTIVE", "", INK); break;
    default: setMessage(frame, "WAITING FOR", "AUTO DATA", MUTED); break;
  }
}

void formatHeading(char *buffer, size_t size, uint16_t centideg)
{
  if (centideg == AUTO_HEADING_INVALID)
    (void)snprintf(buffer, size, "---%c", (char)0xB0);
  else
    (void)snprintf(buffer, size, "%03u%c",
                   (unsigned int)((centideg + 50U) / 100U) % 360U,
                   (char)0xB0);
}

int32_t countToDecidegree(int32_t count)
{
  const int64_t numerator = (int64_t)count * 3600LL;
  const int64_t half_revolution = KEYA_COUNTS_PER_REV / 2;
  return (int32_t)((numerator >= 0LL ? numerator + half_revolution :
                   numerator - half_revolution) / KEYA_COUNTS_PER_REV);
}

void formatMotorAngle(char *buffer, size_t size, int32_t count)
{
  if (count == AUTO_I32_INVALID)
  {
    (void)snprintf(buffer, size, "--.-%c", (char)0xB0);
    return;
  }
  const int32_t tenths = countToDecidegree(count);
  const int32_t magnitude = tenths < 0 ? -tenths : tenths;
  (void)snprintf(buffer, size, "%c%ld.%ld%c",
                 tenths < 0 ? '-' : '+',
                 (long)(magnitude / 10),
                 (long)(magnitude % 10),
                 (char)0xB0);
}

void buildFrame(const DashboardValues &values,
                const AutoDashboardValues &input,
                bool dashboard_can_lost,
                uint32_t now_ms,
                AutoFrame *frame)
{
  const uint32_t timing_started_us = DashboardTiming::beginMeasure();
  memset(frame, 0, sizeof(*frame));
  AutoDashboardValues auto_values = input;
  if (dashboard_can_lost || auto_values.host_valid == 0U)
  {
    auto_values.gps_state = AUTO_GPS_WAIT;
    auto_values.lidar_state = AUTO_STREAM_WAIT;
    auto_values.camera_left_state = AUTO_STREAM_WAIT;
    auto_values.camera_right_state = AUTO_STREAM_WAIT;
    auto_values.drive_state = AUTO_DRIVE_WAIT;
    auto_values.steer_state = AUTO_STEER_WAIT;
    auto_values.brake_state = AUTO_BRAKE_WAIT;
    auto_values.heading_centideg = AUTO_HEADING_INVALID;
    auto_values.target_heading_centideg = AUTO_HEADING_INVALID;
    auto_values.target_speed_centi_kmh = AUTO_I16_INVALID;
    auto_values.actual_motor_count = AUTO_I32_INVALID;
    auto_values.target_motor_count = AUTO_I32_INVALID;
    auto_values.steering_current_deciamp = AUTO_I16_INVALID;
    auto_values.output_state = AUTO_OUTPUT_UNKNOWN;
    auto_values.feedback_state = AUTO_FEEDBACK_UNKNOWN;
  }

  // Native STATUS carries fresh GNSS independently of the host AUTO summary.
  if (!dashboard_can_lost && auto_values.gps_state == AUTO_GPS_WAIT)
  {
    switch (values.gnss_state)
    {
      case GNSS_RTK_FIXED: auto_values.gps_state = AUTO_GPS_FIXED; break;
      case GNSS_FIX: auto_values.gps_state = AUTO_GPS_FIX; break;
      case GNSS_NO_FIX: auto_values.gps_state = AUTO_GPS_NO_FIX; break;
      default: break;
    }
  }
  buildGpsBadge(auto_values.gps_state, &frame->badges[0]);
  buildStreamBadge("LIDAR", auto_values.lidar_state, &frame->badges[1]);
  buildStreamBadge("CAMERA L", auto_values.camera_left_state, &frame->badges[2]);
  buildStreamBadge("CAMERA R", auto_values.camera_right_state, &frame->badges[3]);
  buildDriveBadge(auto_values.drive_state, &frame->badges[4]);
  buildSteerBadge(auto_values.steer_state, &frame->badges[5]);
  setBadge(&frame->badges[6], "E-STOP",
           dashboard_can_lost ? "DATA LOST" :
           ((values.status_flags & STATUS_ESTOP_ACTIVE) != 0U ? "ON" : "OFF"),
           dashboard_can_lost ? UNKNOWN :
           ((values.status_flags & STATUS_ESTOP_ACTIVE) != 0U ? RED : OFF));
  buildBrakeBadge(auto_values.brake_state, &frame->badges[7]);
  if ((dashboard_can_lost || input.host_valid == 0U))
  {
    for (uint8_t index = 0U; index < 8U; ++index)
    {
      if (index == 6U && !dashboard_can_lost) continue;
      if (index == 0U && !dashboard_can_lost &&
          auto_values.gps_state != AUTO_GPS_WAIT) continue;
      (void)snprintf(frame->badges[index].state,
                     sizeof(frame->badges[index].state), "DATA LOST");
      frame->badges[index].background = UNKNOWN;
      frame->badges[index].foreground = WHITE;
    }
  }

  buildMessage(values, input, dashboard_can_lost, frame);
  if (!dashboard_can_lost && (values.status_flags & STATUS_VEHICLE_VALID) != 0U)
  {
    int32_t speed = values.speed_centi_mps;
    if (speed < 0) speed = -speed;
    const uint32_t tenths_kmh = ((uint32_t)speed * 36U + 50U) / 100U;
    (void)snprintf(frame->speed, sizeof(frame->speed), "%lu.%lu",
                   (unsigned long)(tenths_kmh / 10U),
                   (unsigned long)(tenths_kmh % 10U));
  }
  else (void)snprintf(frame->speed, sizeof(frame->speed), "--.-");
  if (auto_values.target_speed_centi_kmh != AUTO_I16_INVALID)
  {
    const int32_t speed = auto_values.target_speed_centi_kmh;
    const int32_t magnitude = speed < 0 ? -speed : speed;
    (void)snprintf(frame->target_speed, sizeof(frame->target_speed), "%s%ld.%ld",
                   speed < 0 ? "-" : "", (long)(magnitude / 100),
                   (long)((magnitude % 100) / 10));
  }
  else (void)snprintf(frame->target_speed, sizeof(frame->target_speed), "--.-");

  formatHeading(frame->heading, sizeof(frame->heading), auto_values.heading_centideg);
  formatHeading(frame->target_heading, sizeof(frame->target_heading),
                auto_values.target_heading_centideg);
  formatMotorAngle(frame->actual_angle, sizeof(frame->actual_angle),
                   auto_values.actual_motor_count);
  formatMotorAngle(frame->target_angle, sizeof(frame->target_angle),
                   auto_values.target_motor_count);
  frame->heading_centideg = auto_values.heading_centideg;
  frame->target_heading_centideg = auto_values.target_heading_centideg;
  frame->actual_motor_count = auto_values.actual_motor_count;
  frame->target_motor_count = auto_values.target_motor_count;
  frame->heading_valid = auto_values.heading_centideg != AUTO_HEADING_INVALID;
  frame->target_heading_valid =
      auto_values.target_heading_centideg != AUTO_HEADING_INVALID;
  frame->actual_motor_valid = auto_values.actual_motor_count != AUTO_I32_INVALID;
  frame->target_motor_valid = auto_values.target_motor_count != AUTO_I32_INVALID;

  if (auto_values.steering_current_deciamp == AUTO_I16_INVALID)
    (void)snprintf(frame->current, sizeof(frame->current), "-- A");
  else
    (void)snprintf(frame->current, sizeof(frame->current), "%ld A",
                   (long)lroundf(auto_values.steering_current_deciamp / 10.0f));
  (void)snprintf(frame->steer_state, sizeof(frame->steer_state), "%s",
                 steerStateText(auto_values.steer_state));
  frame->steer_state_color =
      (auto_values.steer_state == AUTO_STEER_ACTIVE ||
       auto_values.steer_state == AUTO_STEER_WAIT_CMD ||
       auto_values.steer_state == AUTO_STEER_BRAKE ||
       auto_values.steer_state == AUTO_STEER_DISABLED) ? WHITE :
      steerStateColor(auto_values.steer_state);
  (void)snprintf(frame->output, sizeof(frame->output), "%s",
                 auto_values.output_state == AUTO_OUTPUT_ON ? "ON" :
                 auto_values.output_state == AUTO_OUTPUT_OFF ? "OFF" : "--");
  (void)snprintf(frame->feedback, sizeof(frame->feedback), "%s",
                 auto_values.feedback_state == AUTO_FEEDBACK_LIVE ? "LIVE" :
                 auto_values.feedback_state == AUTO_FEEDBACK_LOST ? "LOST" : "--");
  frame->feedback_color =
      auto_values.feedback_state == AUTO_FEEDBACK_LOST ? RED : WHITE;
  prepareFrameGeometry(frame, now_ms);
  DashboardTiming::statePrepared(
      timing_started_us, DashboardTiming::Page::Auto);
}

void drawBadgeRow(uint16_t y, uint8_t index, const Badge &badge)
{
  const uint16_t x0 = (uint16_t)((index % 4U) * 120U);
  const uint16_t y0 = (uint16_t)((index / 4U) * 28U);
  if (y < y0 || y >= y0 + 28U) return;
  fillRow(y, x0, y0, x0 + 119U, y0 + 27U, badge.background);
  if (y == y0 || y == y0 + 27U) fillRow(y, x0, y, x0 + 119U, y, INK);
  else
  {
    row_pixels[x0] = INK;
    row_pixels[x0 + 119U] = INK;
  }
  drawTextRow(y, x0 + 60, y0 + 8, badge.label, auto_font_badge_label,
              badge.foreground, TftHorizontalAnchor::Center);
  drawTextRow(y, x0 + 60, y0 + 21, badge.state, auto_font_badge_state,
              badge.foreground, TftHorizontalAnchor::Center);
}

void drawTraceRow(uint16_t y, const AutoFrame &frame)
{
  bool previous_valid = false;
  float previous_x = 0.0f;
  float previous_y = 0.0f;
  for (uint8_t index = 0U; index < frame.trace_count; ++index)
  {
    const TraceGeometry &point = frame.trace[index];
    if (point.valid == 0U)
    {
      previous_valid = false;
      continue;
    }
    if (previous_valid)
      drawLineRow(y, previous_x, previous_y, point.x, point.y, BLUE, 1U);
    previous_x = point.x;
    previous_y = point.y;
    previous_valid = true;
  }
  drawTextRow(y, 389, 312, frame.trace_peak, auto_font_trace, DARK_TEXT,
              TftHorizontalAnchor::Center);
}

void composeRow(uint16_t y, const AutoFrame &frame, uint32_t now_ms)
{
  (void)now_ms;
  const uint32_t timing_started_us = DashboardTiming::beginMeasure();
  decodeStaticRow(y);
  if (y < 56U)
  {
    const uint8_t first = y < 28U ? 0U : 4U;
    for (uint8_t index = first; index < first + 4U; ++index)
      drawBadgeRow(y, index, frame.badges[index]);
  }

  drawTextRow(y, 240, 66, frame.message_line1, auto_font_message,
              frame.message_color, TftHorizontalAnchor::Center);
  drawTextRow(y, 240, 80, frame.message_line2, auto_font_message,
              frame.message_color, TftHorizontalAnchor::Center);
  drawTextRow(y, 240, 143, frame.speed, auto_font_speed, INK,
              TftHorizontalAnchor::Center);
  drawTextRow(y, 240, 229, frame.target_speed, auto_font_target_speed,
              VIOLET, TftHorizontalAnchor::Center);

  if (y >= 83U && y <= 261U) drawCompassRow(y, frame);
  drawTextRow(y, 8, 225, "NOW", auto_font_badge_label, MUTED,
              TftHorizontalAnchor::Left);
  drawTextRow(y, 153, 225, frame.heading, auto_font_reading, BLUE,
              TftHorizontalAnchor::Right);
  drawTextRow(y, 8, 249, "TARGET", auto_font_badge_label, MUTED,
              TftHorizontalAnchor::Left);
  drawTextRow(y, 153, 249, frame.target_heading, auto_font_reading, VIOLET,
              TftHorizontalAnchor::Right);

  if (y >= 83U && y <= 261U) drawSteeringRow(y, frame);
  drawTextRow(y, 472, 225, frame.actual_angle, auto_font_reading, BLUE,
              TftHorizontalAnchor::Right);
  drawTextRow(y, 472, 249, frame.target_angle, auto_font_reading, VIOLET,
              TftHorizontalAnchor::Right);

  drawTextRow(y, 10, 300, frame.current,
              strlen(frame.current) < 5U ? auto_font_current_large
                                          : auto_font_current_small,
              WHITE, TftHorizontalAnchor::Left);
  drawTextRow(y, 120, 273, frame.steer_state, auto_font_steer_state,
              frame.steer_state_color, TftHorizontalAnchor::Left);
  drawTextRow(y, 256, 292, frame.output, auto_font_message, WHITE,
              TftHorizontalAnchor::Right);
  drawTextRow(y, 256, 309, frame.feedback, auto_font_message,
              frame.feedback_color, TftHorizontalAnchor::Right);
  if (y >= 262U) drawTraceRow(y, frame);
  DashboardTiming::rowComposed(
      timing_started_us, DashboardTiming::Page::Auto, y);
}

Region regionForBit(uint16_t bit)
{
  for (uint8_t index = 0U; index < 8U; ++index)
  {
    if (bit == (uint16_t)(1U << index))
      return {(uint16_t)((index % 4U) * 120U),
              (uint16_t)((index / 4U) * 28U),
              (uint16_t)((index % 4U) * 120U + 119U),
              (uint16_t)((index / 4U) * 28U + 27U)};
  }
  if (bit == DIRTY_MESSAGE) return {160U, 56U, 319U, 88U};
  if (bit == DIRTY_SPEED) return {170U, 89U, 310U, 179U};
  if (bit == DIRTY_TARGET_SPEED) return {185U, 188U, 315U, 261U};
  if (bit == DIRTY_COMPASS) return {0U, 83U, 159U, 261U};
  if (bit == DIRTY_STEERING) return {320U, 83U, 479U, 261U};
  if (bit == DIRTY_STEER_TEXT) return {0U, 262U, 284U, 319U};
  return {285U, 262U, 479U, 319U};
}

void renderRegionRows(const AutoFrame &frame,
                      uint32_t now_ms,
                      const Region &region,
                      uint16_t y0,
                      uint16_t y1)
{
  St7796Lcd::beginPixelStream(region.x0, y0, region.x1, y1);
  for (uint16_t y = y0; y <= y1; ++y)
  {
    composeRow(y, frame, now_ms);
    St7796Lcd::writePixelRow(&row_pixels[region.x0],
                            (size_t)(region.x1 - region.x0 + 1U));
  }
  St7796Lcd::endPixelStream();
}

void recordCurrent(const AutoDashboardValues &values, uint32_t now_ms)
{
  CurrentPoint &point = current_history[current_history_head];
  point.time_ms = now_ms;
  point.deciamp = values.steering_current_deciamp;
  point.valid = values.host_valid != 0U &&
      values.steering_current_deciamp != AUTO_I16_INVALID;
  current_history_head = (current_history_head + 1U) % CURRENT_HISTORY_SIZE;
  if (current_history_count < CURRENT_HISTORY_SIZE) current_history_count++;
}

void markChangedRegions(const AutoFrame &current, const AutoFrame &previous)
{
  for (uint8_t index = 0U; index < 8U; ++index)
  {
    if (memcmp(&current.badges[index], &previous.badges[index], sizeof(Badge)) != 0)
      pending_regions |= (uint16_t)(1U << index);
  }
  if (strcmp(current.message_line1, previous.message_line1) != 0 ||
      strcmp(current.message_line2, previous.message_line2) != 0 ||
      current.message_color != previous.message_color)
    pending_regions |= DIRTY_MESSAGE;
  if (strcmp(current.speed, previous.speed) != 0) pending_regions |= DIRTY_SPEED;
  if (strcmp(current.target_speed, previous.target_speed) != 0)
    pending_regions |= DIRTY_TARGET_SPEED;
  if (current.heading_centideg != previous.heading_centideg ||
      current.target_heading_centideg != previous.target_heading_centideg ||
      strcmp(current.heading, previous.heading) != 0 ||
      strcmp(current.target_heading, previous.target_heading) != 0)
    pending_regions |= DIRTY_COMPASS;
  if (current.actual_motor_count != previous.actual_motor_count ||
      current.target_motor_count != previous.target_motor_count ||
      strcmp(current.actual_angle, previous.actual_angle) != 0 ||
      strcmp(current.target_angle, previous.target_angle) != 0)
    pending_regions |= DIRTY_STEERING;
  if (strcmp(current.current, previous.current) != 0 ||
      strcmp(current.steer_state, previous.steer_state) != 0 ||
      strcmp(current.output, previous.output) != 0 ||
      strcmp(current.feedback, previous.feedback) != 0 ||
      current.steer_state_color != previous.steer_state_color ||
      current.feedback_color != previous.feedback_color)
    pending_regions |= DIRTY_STEER_TEXT;
}

uint16_t selectRoundRobin(uint16_t regions)
{
  for (uint8_t offset = 0U; offset < 15U; ++offset)
  {
    const uint8_t index = (uint8_t)((round_robin_index + offset) % 15U);
    const uint16_t bit = (uint16_t)(1U << index);
    if ((regions & bit) != 0U)
    {
      round_robin_index = (uint8_t)((index + 1U) % 15U);
      return bit;
    }
  }
  return 0U;
}

}  // namespace

namespace AutoTftUi
{

void renderPreparedRows(uint16_t y0, uint16_t y1, uint32_t now_ms)
{
  const Region full = {0U, y0, St7796Lcd::WIDTH - 1U, y1};
  renderRegionRows(desired_frame, now_ms, full, y0, y1);
}

void synchronize(uint32_t now_ms)
{
  previous_queued_frame = desired_frame;
  have_queued_frame = true;
  pending_regions |= DIRTY_ALL;
  urgent_regions = 0U;
  active_region = 0U;
  compose_now_ms = now_ms;
}

void begin(const DashboardValues &values,
           const AutoDashboardValues &auto_values,
           bool dashboard_can_lost,
           uint32_t now_ms)
{
  buildFrame(values, auto_values, dashboard_can_lost, now_ms, &desired_frame);
  renderPreparedRows(0U, St7796Lcd::HEIGHT - 1U, now_ms);
  synchronize(now_ms);
  pending_regions = 0U;
}

void observe(const AutoDashboardValues &auto_values, uint32_t now_ms)
{
  recordCurrent(auto_values, now_ms);
}

void resetState()
{
  memset(current_history, 0, sizeof(current_history));
  current_history_head = 0U;
  current_history_count = 0U;
  have_queued_frame = false;
  pending_regions = 0U;
  urgent_regions = 0U;
  active_region = 0U;
  active_y = 0U;
  round_robin_index = 0U;
  compose_now_ms = 0U;
  last_trace_redraw_ms = 0U;
}

size_t historyCount()
{
  return current_history_count;
}

void queueUpdate(const DashboardValues &values,
                 const AutoDashboardValues &auto_values,
                 bool dashboard_can_lost,
                 uint32_t now_ms,
                 bool urgent_change)
{
  buildFrame(values, auto_values, dashboard_can_lost, now_ms, &desired_frame);
  if (!have_queued_frame)
  {
    previous_queued_frame = desired_frame;
    have_queued_frame = true;
    pending_regions = DIRTY_ALL;
    return;
  }
  markChangedRegions(desired_frame, previous_queued_frame);
  previous_queued_frame = desired_frame;
  if ((uint32_t)(now_ms - last_trace_redraw_ms) >= TRACE_REDRAW_PERIOD_MS)
  {
    pending_regions |= DIRTY_TRACE;
    last_trace_redraw_ms = now_ms;
  }
  compose_now_ms = now_ms;
  if (urgent_change)
  {
    if (active_region != 0U) pending_regions |= active_region;
    active_region = 0U;
    urgent_regions |= pending_regions & URGENT_REGIONS;
  }
}

void service(uint32_t now_ms)
{
  compose_now_ms = now_ms;
  if (active_region == 0U)
  {
    if (pending_regions == 0U) return;
    const uint16_t candidates = urgent_regions != 0U
                                    ? urgent_regions : pending_regions;
    active_region = selectRoundRobin(candidates);
    pending_regions &= (uint16_t)~active_region;
    urgent_regions &= (uint16_t)~active_region;
    active_frame = desired_frame;
    const Region region = regionForBit(active_region);
    active_y = region.y0;
  }
  const Region region = regionForBit(active_region);
  const uint32_t service_started_us = micros();
  uint8_t rendered = 0U;
  while (active_y <= region.y1 && rendered < RENDER_ROWS_PER_SERVICE)
  {
    renderRegionRows(active_frame, compose_now_ms, region, active_y, active_y);
    active_y++;
    rendered++;
    if ((uint32_t)(micros() - service_started_us) >= 2000U) break;
  }
  if (active_y > region.y1)
  {
    DashboardTiming::regionCompleted(
        DashboardTiming::Page::Auto, active_region, millis());
    active_region = 0U;
  }
}

}  // namespace AutoTftUi
