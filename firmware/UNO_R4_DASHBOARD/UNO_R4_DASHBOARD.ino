#include <Arduino_CAN.h>

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "src/include/dashboard_ui.h"
#include "src/include/dashboard_serial.h"
#include "src/include/dashboard_timing.h"
#include "src/include/manual_tft_ui.h"
#include "src/include/st7796_lcd.h"
#include "ft6336_touch.h"
#include "hil_display_report.h"
#include "swipe_test.h"

static constexpr uint8_t DASHBOARD_PROTOCOL_VERSION = 1U;
static constexpr uint32_t DASHBOARD_STATUS_ID = 0x321U;
static constexpr uint32_t DASHBOARD_POWER_ID = 0x322U;
static constexpr uint32_t DASHBOARD_THERMAL_ID = 0x323U;
static constexpr uint32_t DASHBOARD_FAULT_ID = 0x324U;
static constexpr uint32_t DASHBOARD_AUTO_HEALTH_ID = 0x325U;
static constexpr uint32_t DASHBOARD_AUTO_MOTION_ID = 0x326U;
static constexpr uint32_t DASHBOARD_AUTO_ACTUAL_ID = 0x327U;
static constexpr uint32_t DASHBOARD_AUTO_TARGET_ID = 0x328U;
static constexpr uint8_t DASHBOARD_DLC = 8U;

static constexpr uint8_t FRAME_STATUS = 1U << 0;
static constexpr uint8_t FRAME_POWER = 1U << 1;
static constexpr uint8_t FRAME_THERMAL = 1U << 2;
static constexpr uint8_t FRAME_FAULT = 1U << 3;
static constexpr uint8_t FRAME_COMPLETE =
    FRAME_STATUS | FRAME_POWER | FRAME_THERMAL | FRAME_FAULT;
static constexpr uint8_t AUTO_FRAME_HEALTH = 1U << 0;
static constexpr uint8_t AUTO_FRAME_MOTION = 1U << 1;
static constexpr uint8_t AUTO_FRAME_ACTUAL = 1U << 2;
static constexpr uint8_t AUTO_FRAME_TARGET = 1U << 3;
static constexpr uint8_t AUTO_FRAME_COMPLETE =
    AUTO_FRAME_HEALTH | AUTO_FRAME_MOTION | AUTO_FRAME_ACTUAL | AUTO_FRAME_TARGET;
static constexpr uint8_t AUTO_HOST_VALID = 1U << 7;

static constexpr uint32_t CAN_LOST_TIMEOUT_MS = 300U;
static constexpr uint32_t CAN_REINIT_PERIOD_MS = 1000U;
static constexpr uint32_t SERIAL_PRINT_PERIOD_MS = 200U;

struct SnapshotAssembly
{
  DashboardValues values;
  uint8_t alive;
  uint8_t received_frames;
  bool active;
};

struct AutoSnapshotAssembly
{
  AutoDashboardValues values;
  uint8_t alive;
  uint8_t received_frames;
  bool active;
};

static SnapshotAssembly assembly = {};
static AutoSnapshotAssembly auto_assembly = {};
static DashboardValues latest_values = {};
static AutoDashboardValues latest_auto = {};
static uint32_t complete_snapshot_count = 0U;
static uint32_t duplicate_snapshot_count = 0U;
static uint32_t ignored_frame_count = 0U;
static uint32_t can_reinit_count = 0U;
static uint32_t last_complete_snapshot_ms = 0U;
static uint32_t last_complete_auto_snapshot_ms = 0U;
static uint32_t last_can_reinit_ms = 0U;
static uint32_t last_serial_print_ms = 0U;
static uint8_t last_completed_alive = 0U;
static bool has_completed_alive = false;
static bool printed_snapshot = false;
static bool can_lost = false;
static bool can_controller_ready = false;
static bool ui_change_pending = false;
static bool serial_snapshot_pending = false;
static uint8_t serial_event_bits = 0U;
static DashboardValues serial_values = {};
static uint8_t serial_alive = 0U;
static uint8_t serial_line = 0U;
static bool serial_print_active = false;

#if defined(DASHBOARD_BENCH_INPUT)
struct BenchRecord
{
  uint32_t arrival_us;
  uint32_t identifier;
  uint8_t payload[8];
};

static constexpr uint8_t BENCH_QUEUE_SIZE = 32U;
static constexpr uint8_t BENCH_INPUT_BYTES_PER_LOOP = 32U;
static constexpr uint32_t BENCH_EPOCH_LEAD_US = 1000000U;
static BenchRecord bench_records[BENCH_QUEUE_SIZE] = {};
static uint8_t bench_record_count = 0U;
static char bench_line[96] = {};
static uint8_t bench_line_length = 0U;
static bool bench_line_overflow = false;
static bool bench_active = false;
static bool bench_startup_pending = false;
static bool bench_transport_pending = false;
static bool bench_startup_frame_pending = false;
static bool bench_finish_requested = false;
static bool bench_end_enqueued = false;
static uint8_t bench_page = 1U;
static uint32_t bench_duration_ms = 0U;
static uint16_t bench_region_mask = 0U;
static uint32_t bench_epoch_us = 0U;
static uint32_t bench_epoch_lead_us = 0U;
static bool bench_epoch_started = false;
static uint16_t bench_startup_elapsed_ms = 0U;
#endif

static uint16_t get_u16_le(const uint8_t *source)
{
  return (uint16_t)source[0] | ((uint16_t)source[1] << 8);
}

static int16_t get_i16_le(const uint8_t *source)
{
  return (int16_t)get_u16_le(source);
}

static int32_t get_i32_le(const uint8_t *source)
{
  return (int32_t)((uint32_t)source[0] |
                   ((uint32_t)source[1] << 8) |
                   ((uint32_t)source[2] << 16) |
                   ((uint32_t)source[3] << 24));
}

static uint32_t get_u24_le(const uint8_t *source)
{
  return (uint32_t)source[0] |
         ((uint32_t)source[1] << 8) |
         ((uint32_t)source[2] << 16);
}

static bool flag_is_set(uint8_t value, uint8_t flag)
{
  return (value & flag) != 0U;
}

static void format_tenths(char *buffer, size_t size, int16_t value)
{
  const int32_t signed_value = value;
  const uint32_t magnitude =
      (signed_value < 0) ? (uint32_t)(-signed_value) : (uint32_t)signed_value;
  (void)snprintf(buffer, size, "%s%lu.%lu",
                 signed_value < 0 ? "-" : "",
                 (unsigned long)(magnitude / 10U),
                 (unsigned long)(magnitude % 10U));
}

static void format_hundredths(char *buffer, size_t size, int16_t value)
{
  const int32_t signed_value = value;
  const uint32_t magnitude =
      (signed_value < 0) ? (uint32_t)(-signed_value) : (uint32_t)signed_value;
  (void)snprintf(buffer, size, "%s%lu.%02lu",
                 signed_value < 0 ? "-" : "",
                 (unsigned long)(magnitude / 100U),
                 (unsigned long)(magnitude % 100U));
}

static void format_display_kmh(char *buffer, size_t size, int16_t speed_centi_mps)
{
  const int32_t signed_speed = speed_centi_mps;
  const uint32_t magnitude =
      (signed_speed < 0) ? (uint32_t)(-signed_speed) : (uint32_t)signed_speed;
  const uint32_t tenths_kmh = ((magnitude * 36U) + 50U) / 100U;
  (void)snprintf(buffer, size, "%lu.%lu",
                 (unsigned long)(tenths_kmh / 10U),
                 (unsigned long)(tenths_kmh % 10U));
}

static const char *yes_no(bool value)
{
  return value ? "YES" : "NO";
}

static const char *gnss_state_text(uint8_t state)
{
  switch (state)
  {
    case GNSS_NO_FIX:
      return "NO_FIX";
    case GNSS_FIX:
      return "FIX";
    case GNSS_RTK_FIXED:
      return "RTK_FIXED";
    default:
      return "UNAVAILABLE";
  }
}

static void build_snapshot_line(char *buffer,
                                size_t size,
                                const DashboardValues &values,
                                uint8_t alive,
                                uint8_t line)
{
  const bool vehicle_valid =
      flag_is_set(values.status_flags, STATUS_VEHICLE_VALID);
  const bool detail_valid =
      flag_is_set(values.status_flags, STATUS_DETAIL_VALID);
  const bool part1_valid =
      flag_is_set(values.status_flags, STATUS_PART1_VALID);
  const bool part2_valid =
      flag_is_set(values.status_flags, STATUS_PART2_VALID);

  if (line == 0U)
  {
    (void)snprintf(buffer, size,
                   "DASH v1 alive=%u complete=%lu ignored=%lu duplicate=%lu reinit=%lu",
                   alive, (unsigned long)complete_snapshot_count,
                   (unsigned long)ignored_frame_count,
                   (unsigned long)duplicate_snapshot_count,
                   (unsigned long)can_reinit_count);
  }
  else if (line == 1U)
  {
    (void)snprintf(buffer, size, "  LINK vehicle=%s detail=%s part1=%s part2=%s",
                   vehicle_valid ? "OK" : "LOST",
                   detail_valid ? "OK" : "LOST",
                   part1_valid ? "VALID" : "INVALID",
                   part2_valid ? "VALID" : "INVALID");
  }
  else if (line == 2U)
  {
    if (!vehicle_valid)
      (void)snprintf(buffer, size, "  GNSS state=%s MOTION mode=-- gear=-- speed=--",
                     gnss_state_text(values.gnss_state));
    else
    {
      char speed[16];
      char display[16];
      format_hundredths(speed, sizeof(speed), values.speed_centi_mps);
      format_display_kmh(display, sizeof(display), values.speed_centi_mps);
      (void)snprintf(buffer, size,
                     "  GNSS state=%s MOTION mode=%s gear=%s speed=%sm/s display=%skm/h",
                     gnss_state_text(values.gnss_state),
                     flag_is_set(values.status_flags, STATUS_MODE_AUTO) ? "AUTO" : "MANUAL",
                     flag_is_set(values.status_flags, STATUS_GEAR_REVERSE) ? "REV" : "FWD",
                     speed, display);
    }
  }
  else if (line == 3U)
  {
    if (!detail_valid)
      (void)snprintf(buffer, size,
                     "  SAFETY estop=%s brake=%s DRIVE pedal=-- tractive=-- command=-- stop=--",
                     yes_no(flag_is_set(values.status_flags, STATUS_ESTOP_ACTIVE)),
                     yes_no(flag_is_set(values.status_flags, STATUS_BRAKE_ACTIVE)));
    else
      (void)snprintf(buffer, size,
                     "  SAFETY estop=%s brake=%s DRIVE pedal=%u%% tractive=%s command=%s stop=%s",
                     yes_no(flag_is_set(values.status_flags, STATUS_ESTOP_ACTIVE)),
                     yes_no(flag_is_set(values.status_flags, STATUS_BRAKE_ACTIVE)),
                     values.pedal_percent,
                     flag_is_set(values.drive_flags, DRIVE_TRACTIVE_FEEDBACK) ? "RUN" : "STOP",
                     flag_is_set(values.drive_flags, DRIVE_COMMAND_TIMEOUT) ? "TIMEOUT" :
                     flag_is_set(values.drive_flags, DRIVE_COMMAND_RECEIVED) ? "OK" : "WAIT",
                     yes_no(flag_is_set(values.drive_flags, DRIVE_STOP)));
  }
  else if (line == 4U)
  {
    if (!detail_valid || !part1_valid)
      (void)snprintf(buffer, size, "  POWER voltage=-- bus_current=-- phase_current=--");
    else
    {
      char bus[16];
      char phase[16];
      format_tenths(bus, sizeof(bus), values.bus_current_deciamp);
      format_tenths(phase, sizeof(phase), values.phase_current_deciamp);
      (void)snprintf(buffer, size, "  POWER voltage=%u.%uV bus=%sA phase=%sA",
                     values.bus_voltage_decivolt / 10U,
                     values.bus_voltage_decivolt % 10U, bus, phase);
    }
  }
  else if (line == 5U)
  {
    if (!detail_valid || !part2_valid)
      (void)snprintf(buffer, size, "  THERMAL controller=-- motor=-- fault=--");
    else
    {
      char controller[16];
      char motor[16];
      format_tenths(controller, sizeof(controller), values.controller_temperature_decic);
      format_tenths(motor, sizeof(motor), values.motor_temperature_decic);
      (void)snprintf(buffer, size, "  THERMAL controller=%sC motor=%sC fault=0x%06lX",
                     controller, motor,
                     (unsigned long)(values.motor_fault_bits & 0x00FFFFFFUL));
    }
  }
}

static void service_serial_output(uint32_t now)
{
  static const char *const event_messages[] = {
      "CAN LOST: no fresh complete snapshot for 300 ms",
      "CAN RECOVERED",
      "CAN controller reinitialized; waiting for fresh alive",
      "CAN controller reinitialize failed"};
  if (serial_event_bits != 0U && DashboardSerial::hasCapacity())
  {
    for (uint8_t index = 0U; index < 4U; ++index)
    {
      const uint8_t bit = (uint8_t)(1U << index);
      if ((serial_event_bits & bit) != 0U &&
          DashboardSerial::enqueue(event_messages[index]))
      {
        serial_event_bits &= (uint8_t)~bit;
        break;
      }
    }
  }
#if defined(DASHBOARD_BENCH_INPUT)
  if (bench_active) return;
#endif
  if (!printed_snapshot ||
      (uint32_t)(now - last_serial_print_ms) >= SERIAL_PRINT_PERIOD_MS)
  {
    if (serial_snapshot_pending && serial_line == 0U)
    {
      serial_values = latest_values;
      serial_alive = last_completed_alive;
      serial_snapshot_pending = false;
      printed_snapshot = true;
      last_serial_print_ms = now;
      serial_print_active = true;
    }
  }
  if (serial_print_active && DashboardSerial::hasCapacity())
  {
    char line[DashboardSerial::MESSAGE_SIZE];
    build_snapshot_line(
        line, sizeof(line), serial_values, serial_alive, serial_line);
    if (!DashboardSerial::enqueue(line)) return;
    if (serial_line >= 5U)
    {
      serial_line = 0U;
      serial_print_active = false;
    }
    else serial_line++;
  }
}

static void start_snapshot(uint8_t alive)
{
  assembly.values = {};
  assembly.alive = alive;
  assembly.received_frames = 0U;
  assembly.active = true;
}

static void start_auto_snapshot(uint8_t alive)
{
  auto_assembly.values = {};
  auto_assembly.values.heading_centideg = AUTO_HEADING_INVALID;
  auto_assembly.values.target_heading_centideg = AUTO_HEADING_INVALID;
  auto_assembly.values.target_speed_centi_kmh = AUTO_I16_INVALID;
  auto_assembly.values.actual_motor_count = AUTO_I32_INVALID;
  auto_assembly.values.target_motor_count = AUTO_I32_INVALID;
  auto_assembly.values.steering_current_deciamp = AUTO_I16_INVALID;
  auto_assembly.alive = alive;
  auto_assembly.received_frames = 0U;
  auto_assembly.active = true;
}

static bool is_dashboard_id(uint32_t identifier)
{
  return (identifier == DASHBOARD_STATUS_ID) ||
         (identifier == DASHBOARD_POWER_ID) ||
         (identifier == DASHBOARD_THERMAL_ID) ||
         (identifier == DASHBOARD_FAULT_ID) ||
         (identifier == DASHBOARD_AUTO_HEALTH_ID) ||
         (identifier == DASHBOARD_AUTO_MOTION_ID) ||
         (identifier == DASHBOARD_AUTO_ACTUAL_ID) ||
         (identifier == DASHBOARD_AUTO_TARGET_ID);
}

static bool is_auto_dashboard_id(uint32_t identifier)
{
  return identifier >= DASHBOARD_AUTO_HEALTH_ID &&
         identifier <= DASHBOARD_AUTO_TARGET_ID;
}

static void accept_auto_dashboard_frame(const CanMsg &message)
{
  const uint32_t identifier = message.getStandardId();
  const uint8_t alive = message.data[1];
  if (!auto_assembly.active || auto_assembly.alive != alive)
  {
    start_auto_snapshot(alive);
  }

  if (identifier == DASHBOARD_AUTO_HEALTH_ID)
  {
    auto_assembly.values.gps_state = message.data[2] & 0x0FU;
    auto_assembly.values.lidar_state = message.data[2] >> 4;
    auto_assembly.values.camera_left_state = message.data[3] & 0x0FU;
    auto_assembly.values.camera_right_state = message.data[3] >> 4;
    auto_assembly.values.drive_state = message.data[4] & 0x0FU;
    auto_assembly.values.steer_state = message.data[4] >> 4;
    auto_assembly.values.brake_state = message.data[5] & 0x0FU;
    auto_assembly.values.activity = message.data[5] >> 4;
    auto_assembly.values.output_state = message.data[6] & 0x0FU;
    auto_assembly.values.feedback_state = message.data[6] >> 4;
    auto_assembly.values.host_valid =
        (message.data[7] & AUTO_HOST_VALID) != 0U;
    auto_assembly.received_frames |= AUTO_FRAME_HEALTH;
  }
  else if (identifier == DASHBOARD_AUTO_MOTION_ID)
  {
    auto_assembly.values.heading_centideg = get_u16_le(&message.data[2]);
    auto_assembly.values.target_heading_centideg = get_u16_le(&message.data[4]);
    auto_assembly.values.target_speed_centi_kmh = get_i16_le(&message.data[6]);
    auto_assembly.received_frames |= AUTO_FRAME_MOTION;
  }
  else if (identifier == DASHBOARD_AUTO_ACTUAL_ID)
  {
    auto_assembly.values.actual_motor_count = get_i32_le(&message.data[2]);
    auto_assembly.values.steering_current_deciamp = get_i16_le(&message.data[6]);
    auto_assembly.received_frames |= AUTO_FRAME_ACTUAL;
  }
  else
  {
    auto_assembly.values.target_motor_count = get_i32_le(&message.data[2]);
    auto_assembly.received_frames |= AUTO_FRAME_TARGET;
  }

  if (auto_assembly.received_frames == AUTO_FRAME_COMPLETE)
  {
    latest_auto = auto_assembly.values;
    last_complete_auto_snapshot_ms = millis();
    DashboardUi::observeAuto(latest_auto, last_complete_auto_snapshot_ms);
    auto_assembly.active = false;
    ui_change_pending = true;
  }
}

static void accept_dashboard_frame(const CanMsg &message)
{
  if (!message.isStandardId() ||
      !is_dashboard_id(message.getStandardId()) ||
      (message.data_length != DASHBOARD_DLC) ||
      (message.data[0] != DASHBOARD_PROTOCOL_VERSION))
  {
    ignored_frame_count++;
    return;
  }

  const uint32_t identifier = message.getStandardId();
  if (is_auto_dashboard_id(identifier))
  {
    accept_auto_dashboard_frame(message);
    return;
  }
  const uint8_t alive = message.data[1];
  if (!assembly.active || (assembly.alive != alive))
  {
    start_snapshot(alive);
  }

  if (identifier == DASHBOARD_STATUS_ID)
  {
    assembly.values.status_flags = message.data[2];
    assembly.values.drive_flags = message.data[3];
    assembly.values.pedal_percent = message.data[4];
    assembly.values.gnss_state =
        (message.data[5] <= GNSS_RTK_FIXED) ? message.data[5]
                                            : GNSS_UNAVAILABLE;
    assembly.values.speed_centi_mps = get_i16_le(&message.data[6]);
    assembly.received_frames |= FRAME_STATUS;
  }
  else if (identifier == DASHBOARD_POWER_ID)
  {
    assembly.values.bus_voltage_decivolt = get_u16_le(&message.data[2]);
    assembly.values.bus_current_deciamp = get_i16_le(&message.data[4]);
    assembly.values.phase_current_deciamp = get_i16_le(&message.data[6]);
    assembly.received_frames |= FRAME_POWER;
  }
  else if (identifier == DASHBOARD_THERMAL_ID)
  {
    assembly.values.controller_temperature_decic =
        get_i16_le(&message.data[2]);
    assembly.values.motor_temperature_decic = get_i16_le(&message.data[4]);
    assembly.received_frames |= FRAME_THERMAL;
  }
  else
  {
    assembly.values.motor_fault_bits = get_u24_le(&message.data[2]);
    assembly.received_frames |= FRAME_FAULT;
  }

  if (assembly.received_frames == FRAME_COMPLETE)
  {
    const uint32_t now = millis();
    if (!has_completed_alive || (assembly.alive != last_completed_alive))
    {
      has_completed_alive = true;
      last_completed_alive = assembly.alive;
      last_complete_snapshot_ms = now;
      complete_snapshot_count++;
      latest_values = assembly.values;
      if (can_lost)
      {
        can_lost = false;
        serial_event_bits |= 1U << 1;
      }
      ui_change_pending = true;
      serial_snapshot_pending = true;
    }
    else
    {
      duplicate_snapshot_count++;
    }
    assembly.active = false;
  }
}

static void drain_can_receive_buffer()
{
  while (CAN.available() > 0U)
  {
    (void)CAN.read();
  }
}

static void reset_receiver_state()
{
  assembly = {};
  auto_assembly = {};
  latest_values = {};
  start_auto_snapshot(0U);
  latest_auto = auto_assembly.values;
  auto_assembly.active = false;
  complete_snapshot_count = 0U;
  duplicate_snapshot_count = 0U;
  ignored_frame_count = 0U;
  can_reinit_count = 0U;
  last_complete_snapshot_ms = 0U;
  last_complete_auto_snapshot_ms = 0U;
  last_can_reinit_ms = 0U;
  last_serial_print_ms = 0U;
  last_completed_alive = 0U;
  has_completed_alive = false;
  printed_snapshot = false;
  can_lost = false;
  can_controller_ready = false;
  ui_change_pending = false;
  serial_snapshot_pending = false;
  serial_event_bits = 0U;
  serial_line = 0U;
  serial_print_active = false;
  DashboardUi::resetState();
}

#if defined(DASHBOARD_BENCH_INPUT)
static void bench_error(const char *message)
{
  char line[DashboardSerial::MESSAGE_SIZE];
  (void)snprintf(line, sizeof(line), "BENCH ERROR %s", message);
  (void)DashboardSerial::enqueue(line);
}

static int8_t hex_nibble(char value)
{
  if (value >= '0' && value <= '9') return (int8_t)(value - '0');
  if (value >= 'a' && value <= 'f') return (int8_t)(value - 'a' + 10);
  if (value >= 'A' && value <= 'F') return (int8_t)(value - 'A' + 10);
  return -1;
}

static void reset_bench_run(uint8_t page)
{
  reset_receiver_state();
  bench_record_count = 0U;
  if (page == 0U || page >= 3U)
  {
    St7796Lcd::setBacklight(true);
    St7796Lcd::fillScreen(St7796Lcd::COLOR_BLACK);
  }
  else
  {
    DashboardUi::begin(latest_values, latest_auto, true);
  }
  last_complete_snapshot_ms = millis();
  last_complete_auto_snapshot_ms = millis();
  last_can_reinit_ms = millis();
}

static void begin_bench_run(uint8_t page,
                            uint32_t duration_ms,
                            uint16_t region_mask)
{
  reset_bench_run(page);
  bench_page = page;
  bench_duration_ms = duration_ms;
  bench_region_mask = region_mask;
  bench_finish_requested = false;
  bench_end_enqueued = false;
  bench_startup_pending = page == 0U;
  bench_transport_pending = page == 3U;
  bench_startup_frame_pending = page == 4U;
  bench_startup_elapsed_ms = page == 4U ? region_mask : 0U;
  bench_epoch_started = false;
  bench_active = true;
  bench_epoch_lead_us =
      (page == 1U || page == 2U) ? BENCH_EPOCH_LEAD_US : 0U;
  bench_epoch_us = micros() + bench_epoch_lead_us;
  char line[DashboardSerial::MESSAGE_SIZE];
  (void)snprintf(line, sizeof(line),
                 "BENCH READY epoch_us=%lu lead_us=%lu pad=00000000000000000000000000000000",
                 (unsigned long)bench_epoch_us,
                 (unsigned long)bench_epoch_lead_us);
  (void)DashboardSerial::enqueue(line);
}

static void parse_bench_line(const char *line)
{
  if (line[0] == 'B' && line[1] == ' ')
  {
    unsigned int page = 0U;
    unsigned long duration_ms = 0UL;
    unsigned int region_mask = 0U;
    char extra = '\0';
    if (sscanf(line, "B %u %lu %x %c",
               &page, &duration_ms, &region_mask, &extra) != 3 ||
        page > 3U || duration_ms == 0UL || duration_ms > 60000UL ||
        region_mask > 0xFFFFU)
    {
      bench_error("malformed B");
      return;
    }
    begin_bench_run((uint8_t)page, (uint32_t)duration_ms,
                    (uint16_t)region_mask);
    return;
  }
  if (line[0] == 'F' && line[1] == ' ')
  {
    unsigned int elapsed_ms = 0U;
    char extra = '\0';
    if (sscanf(line, "F %u %c", &elapsed_ms, &extra) != 1 ||
        elapsed_ms > 1400U)
    {
      bench_error("malformed F");
      return;
    }
    begin_bench_run(4U, 5000U, (uint16_t)elapsed_ms);
    return;
  }
  if (strcmp(line, "E") == 0)
  {
    if (!bench_active) bench_error("E without run");
    else bench_finish_requested = true;
    return;
  }
  if (line[0] == 'C' && line[1] == ' ')
  {
    unsigned long arrival_us = 0UL;
    unsigned long identifier = 0UL;
    char payload_hex[17] = {};
    char extra = '\0';
    if (!bench_active || (bench_page != 1U && bench_page != 2U) ||
        sscanf(line, "C %lu %lx %16s %c",
               &arrival_us, &identifier, payload_hex, &extra) != 3 ||
        strlen(payload_hex) != 16U || identifier > 0x7FFUL ||
        arrival_us > bench_duration_ms * 1000UL || bench_finish_requested)
    {
      bench_error("malformed C");
      return;
    }
    if (bench_record_count >= BENCH_QUEUE_SIZE)
    {
      bench_error("queue overflow");
      return;
    }
    const int32_t epoch_delta_us = (int32_t)(micros() - bench_epoch_us);
    if (epoch_delta_us >= 0 && (uint32_t)arrival_us <= (uint32_t)epoch_delta_us)
    {
      char message[DashboardSerial::MESSAGE_SIZE];
      (void)snprintf(message, sizeof(message),
                     "late C arrival_us=%lu elapsed_us=%lu",
                     arrival_us, (unsigned long)(uint32_t)epoch_delta_us);
      bench_error(message);
      return;
    }
    if (bench_record_count != 0U &&
        (uint32_t)arrival_us < bench_records[bench_record_count - 1U].arrival_us)
    {
      bench_error("out of order C");
      return;
    }
    BenchRecord &record = bench_records[bench_record_count];
    record.arrival_us = (uint32_t)arrival_us;
    record.identifier = (uint32_t)identifier;
    for (uint8_t index = 0U; index < 8U; ++index)
    {
      const int8_t high = hex_nibble(payload_hex[index * 2U]);
      const int8_t low = hex_nibble(payload_hex[index * 2U + 1U]);
      if (high < 0 || low < 0)
      {
        bench_error("payload hex");
        return;
      }
      record.payload[index] = (uint8_t)((high << 4) | low);
    }
    bench_record_count++;
    return;
  }
  bench_error("unknown command");
}

static void service_bench_input()
{
  uint8_t consumed = 0U;
  while (consumed < BENCH_INPUT_BYTES_PER_LOOP && Serial.available() > 0)
  {
    if (bench_record_count >= BENCH_QUEUE_SIZE && bench_line_length == 0U)
      return;
    const int input = Serial.read();
    if (input < 0) break;
    consumed++;
    const char value = (char)input;
    if (value == '\r') continue;
    if (value == '\n')
    {
      if (bench_line_overflow) bench_error("line too long");
      else if (bench_line_length != 0U)
      {
        bench_line[bench_line_length] = '\0';
        parse_bench_line(bench_line);
      }
      bench_line_length = 0U;
      bench_line_overflow = false;
    }
    else if (bench_line_length + 1U < sizeof(bench_line))
      bench_line[bench_line_length++] = value;
    else
      bench_line_overflow = true;
  }
}

static uint8_t bench_ready_count()
{
  if (!bench_active || !bench_epoch_started || bench_startup_pending ||
      bench_finish_requested) return 0U;
  const uint32_t elapsed_us = (uint32_t)(micros() - bench_epoch_us);
  uint8_t ready = 0U;
  while (ready < bench_record_count &&
         bench_records[ready].arrival_us <= elapsed_us)
    ready++;
  return ready;
}

static void process_bench_records(uint8_t ready)
{
  for (uint8_t index = 0U; index < ready; ++index)
  {
    const BenchRecord &record = bench_records[index];
    const CanMsg message(record.identifier, 8U, record.payload);
    accept_dashboard_frame(message);
  }
  if (ready != 0U)
  {
    memmove(bench_records, &bench_records[ready],
            (bench_record_count - ready) * sizeof(BenchRecord));
    bench_record_count = (uint8_t)(bench_record_count - ready);
  }
}

static void service_bench_run()
{
  if (!bench_active) return;
  if (!bench_epoch_started && (int32_t)(micros() - bench_epoch_us) >= 0)
  {
    bench_epoch_started = true;
    last_complete_snapshot_ms = millis();
    last_complete_auto_snapshot_ms = millis();
    last_can_reinit_ms = millis();
    if (bench_page == 1U || bench_page == 2U)
      DashboardTiming::beginBench(
          (DashboardTiming::Page)bench_page, bench_duration_ms,
          bench_region_mask, millis());
    else if (bench_page == 3U)
      DashboardTiming::beginBench(
          DashboardTiming::Page::Transport, bench_duration_ms, 0U, millis());
    else if (bench_page == 4U)
      DashboardTiming::beginBench(
          DashboardTiming::Page::StartupFixed, bench_duration_ms,
          bench_startup_elapsed_ms, millis());
  }
  if (bench_epoch_started && bench_startup_pending && DashboardSerial::idle())
  {
    bench_startup_pending = false;
    DashboardTiming::beginBench(
        DashboardTiming::Page::Startup, bench_duration_ms,
        bench_region_mask, millis());
    ManualTftUi::playStartup();
    DashboardTiming::regionCompleted(
        DashboardTiming::Page::Startup, 1U, millis());
    DashboardTiming::endSegment();
    DashboardUi::begin(latest_values, latest_auto, true);
    bench_finish_requested = true;
  }
  if (bench_epoch_started && bench_transport_pending && DashboardSerial::idle())
  {
    bench_transport_pending = false;
#if defined(DASHBOARD_TIMING_TRANSPORT_PROFILE)
    St7796Lcd::runTransportCalibration();
#endif
    DashboardTiming::endSegment();
    bench_finish_requested = true;
  }
  if (bench_epoch_started && bench_startup_frame_pending &&
      DashboardSerial::idle())
  {
    bench_startup_frame_pending = false;
#if defined(DASHBOARD_TIMING_STARTUP_PROFILE)
    ManualTftUi::renderStartupFrameForTiming(bench_startup_elapsed_ms);
#endif
    DashboardTiming::endSegment();
    bench_finish_requested = true;
  }
  if (bench_epoch_started && !bench_startup_pending && !bench_finish_requested &&
      (uint32_t)(micros() - bench_epoch_us) >= bench_duration_ms * 1000UL)
    bench_finish_requested = true;
  if (bench_finish_requested)
  {
    DashboardTiming::endSegment();
    if (!bench_end_enqueued && DashboardTiming::reportDone() &&
        DashboardSerial::hasCapacity())
    {
      char line[DashboardSerial::MESSAGE_SIZE];
      (void)snprintf(line, sizeof(line), "BENCH END elapsed_us=%lu",
                     (unsigned long)(uint32_t)(micros() - bench_epoch_us));
      bench_end_enqueued = DashboardSerial::enqueue(line);
    }
    if (bench_end_enqueued && DashboardSerial::idle())
    {
      bench_active = false;
      serial_snapshot_pending = false;
      serial_print_active = false;
    }
  }
}
#endif

static void try_reinitialize_can()
{
  if (can_controller_ready)
  {
    drain_can_receive_buffer();
  }
  CAN.end();
  can_controller_ready = false;

  assembly.active = false;
  auto_assembly.active = false;
  if (CAN.begin(CanBitRate::BR_250k))
  {
    can_controller_ready = true;
    has_completed_alive = false;
    can_reinit_count++;
    drain_can_receive_buffer();
    serial_event_bits |= 1U << 2;
  }
  else
  {
    serial_event_bits |= 1U << 3;
  }
}

void setup()
{
  Serial.begin(115200);
  reset_receiver_state();

  St7796Lcd::begin();
  St7796Lcd::setBacklight(true);
  DashboardTiming::beginSegment(
      millis(), DashboardTiming::Page::Startup);
  ManualTftUi::playStartup();
  DashboardTiming::endSegment();
  Ft6336Touch::begin();
  start_auto_snapshot(0U);
  latest_auto = auto_assembly.values;
  auto_assembly.active = false;
  DashboardUi::begin(latest_values, latest_auto, true);
#if defined(DASHBOARD_BENCH_INPUT)
  can_controller_ready = false;
#else
  can_controller_ready = CAN.begin(CanBitRate::BR_250k);
#endif
  (void)DashboardSerial::enqueue("LCD dashboard UI rendered");
  SwipeTest::setEnabled(true);

  if (!can_controller_ready)
  {
    (void)DashboardSerial::enqueue("CAN.begin(BR_250k) failed");
  }

  last_complete_snapshot_ms = millis();
  last_can_reinit_ms = last_complete_snapshot_ms;
  (void)DashboardSerial::enqueue(
      "UNO R4 dashboard CAN protocol v1 receiver ready");
  (void)DashboardSerial::enqueue(
      "Waiting for standard IDs 0x321-0x328, DLC=8");
}

void loop()
{
#if defined(DASHBOARD_BENCH_INPUT)
  const uint8_t bench_ready_at_loop_start = bench_ready_count();
  process_bench_records(bench_ready_at_loop_start);
  service_bench_input();
#else
  const int received_at_loop_start = can_controller_ready ? CAN.available() : 0;
  for (int index = 0;
       index < received_at_loop_start && CAN.available() > 0U;
       ++index)
  {
    accept_dashboard_frame(CAN.read());
  }
#endif

  const uint32_t now = millis();
#if defined(DASHBOARD_BENCH_INPUT)
  const bool bench_startup_active =
      bench_active && (bench_page == 0U || bench_page >= 3U);
  const bool bench_waiting_epoch = bench_active && !bench_epoch_started;
#else
  const bool bench_startup_active = false;
  const bool bench_waiting_epoch = false;
#endif
  if (!bench_startup_active && !bench_waiting_epoch)
  {
  if (latest_auto.host_valid != 0U &&
      (uint32_t)(now - last_complete_auto_snapshot_ms) >= CAN_LOST_TIMEOUT_MS)
  {
    latest_auto.host_valid = 0U;
    DashboardUi::observeAuto(latest_auto, now);
    ui_change_pending = true;
  }
  if (!can_lost &&
      ((uint32_t)(now - last_complete_snapshot_ms) >= CAN_LOST_TIMEOUT_MS))
  {
    serial_event_bits |= 1U << 0;
    can_lost = true;
    assembly.active = false;
    ui_change_pending = true;
  }

  if (ui_change_pending)
  {
    DashboardUi::update(
        latest_values, latest_auto, can_lost, last_completed_alive);
    ui_change_pending = false;
  }
  }

#if !defined(DASHBOARD_BENCH_INPUT)
  if (can_lost &&
      ((uint32_t)(now - last_can_reinit_ms) >= CAN_REINIT_PERIOD_MS))
  {
    last_can_reinit_ms = now;
    try_reinitialize_can();
  }
#endif
  Ft6336Touch::poll(now);
  SwipeTest::poll(now);
  DashboardUi::service(now);
#if defined(DASHBOARD_BENCH_INPUT)
  service_bench_run();
#endif
  HilDisplayReport::poll(
      now, complete_snapshot_count,
      complete_snapshot_count ? (int32_t)(now - last_complete_snapshot_ms) : -1);
  if (!HilDisplayReport::enabled())
  {
    service_serial_output(now);
    DashboardTiming::serviceReport(now);
    DashboardSerial::service();
  }
}
