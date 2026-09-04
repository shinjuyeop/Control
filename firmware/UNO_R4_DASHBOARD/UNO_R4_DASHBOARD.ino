#include <Arduino_CAN.h>

#include <stdint.h>

#include "dashboard_ui.h"
#include "manual_tft_ui.h"
#include "st7796_lcd.h"

static constexpr uint8_t DASHBOARD_PROTOCOL_VERSION = 1U;
static constexpr uint32_t DASHBOARD_STATUS_ID = 0x321U;
static constexpr uint32_t DASHBOARD_POWER_ID = 0x322U;
static constexpr uint32_t DASHBOARD_THERMAL_ID = 0x323U;
static constexpr uint32_t DASHBOARD_FAULT_ID = 0x324U;
static constexpr uint8_t DASHBOARD_DLC = 8U;

static constexpr uint8_t FRAME_STATUS = 1U << 0;
static constexpr uint8_t FRAME_POWER = 1U << 1;
static constexpr uint8_t FRAME_THERMAL = 1U << 2;
static constexpr uint8_t FRAME_FAULT = 1U << 3;
static constexpr uint8_t FRAME_COMPLETE =
    FRAME_STATUS | FRAME_POWER | FRAME_THERMAL | FRAME_FAULT;

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

static SnapshotAssembly assembly = {};
static DashboardValues latest_values = {};
static uint32_t complete_snapshot_count = 0U;
static uint32_t duplicate_snapshot_count = 0U;
static uint32_t ignored_frame_count = 0U;
static uint32_t can_reinit_count = 0U;
static uint32_t last_complete_snapshot_ms = 0U;
static uint32_t last_can_reinit_ms = 0U;
static uint32_t last_serial_print_ms = 0U;
static uint8_t last_completed_alive = 0U;
static bool has_completed_alive = false;
static bool printed_snapshot = false;
static bool can_lost = false;
static bool can_controller_ready = false;

static uint16_t get_u16_le(const uint8_t *source)
{
  return (uint16_t)source[0] | ((uint16_t)source[1] << 8);
}

static int16_t get_i16_le(const uint8_t *source)
{
  return (int16_t)get_u16_le(source);
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

static void print_tenths(int16_t value)
{
  const int32_t signed_value = value;
  const uint32_t magnitude =
      (signed_value < 0) ? (uint32_t)(-signed_value) : (uint32_t)signed_value;
  if (signed_value < 0) Serial.print('-');
  Serial.print(magnitude / 10U);
  Serial.print('.');
  Serial.print(magnitude % 10U);
}

static void print_hundredths(int16_t value)
{
  const int32_t signed_value = value;
  const uint32_t magnitude =
      (signed_value < 0) ? (uint32_t)(-signed_value) : (uint32_t)signed_value;
  if (signed_value < 0) Serial.print('-');
  Serial.print(magnitude / 100U);
  Serial.print('.');
  if ((magnitude % 100U) < 10U) Serial.print('0');
  Serial.print(magnitude % 100U);
}

static void print_display_kmh(int16_t speed_centi_mps)
{
  const int32_t signed_speed = speed_centi_mps;
  const uint32_t magnitude =
      (signed_speed < 0) ? (uint32_t)(-signed_speed) : (uint32_t)signed_speed;
  const uint32_t tenths_kmh = ((magnitude * 36U) + 50U) / 100U;
  Serial.print(tenths_kmh / 10U);
  Serial.print('.');
  Serial.print(tenths_kmh % 10U);
}

static void print_hex24(uint32_t value)
{
  value &= 0x00FFFFFFUL;
  if (value < 0x100000UL) Serial.print('0');
  if (value < 0x010000UL) Serial.print('0');
  if (value < 0x001000UL) Serial.print('0');
  if (value < 0x000100UL) Serial.print('0');
  if (value < 0x000010UL) Serial.print('0');
  Serial.print(value, HEX);
}

static const __FlashStringHelper *yes_no(bool value)
{
  return value ? F("YES") : F("NO");
}

static const __FlashStringHelper *gnss_state_text(uint8_t state)
{
  switch (state)
  {
    case GNSS_NO_FIX:
      return F("NO_FIX");
    case GNSS_FIX:
      return F("FIX");
    case GNSS_RTK_FIXED:
      return F("RTK_FIXED");
    default:
      return F("UNAVAILABLE");
  }
}

static void print_snapshot(const DashboardValues &values, uint8_t alive)
{
  const bool vehicle_valid =
      flag_is_set(values.status_flags, STATUS_VEHICLE_VALID);
  const bool detail_valid =
      flag_is_set(values.status_flags, STATUS_DETAIL_VALID);
  const bool part1_valid =
      flag_is_set(values.status_flags, STATUS_PART1_VALID);
  const bool part2_valid =
      flag_is_set(values.status_flags, STATUS_PART2_VALID);

  Serial.print(F("DASH v1 alive="));
  Serial.print(alive);
  Serial.print(F(" complete="));
  Serial.print(complete_snapshot_count);
  Serial.print(F(" ignored="));
  Serial.print(ignored_frame_count);
  Serial.print(F(" duplicate="));
  Serial.print(duplicate_snapshot_count);
  Serial.print(F(" reinit="));
  Serial.println(can_reinit_count);

  Serial.print(F("  LINK vehicle="));
  Serial.print(vehicle_valid ? F("OK") : F("LOST"));
  Serial.print(F(" detail="));
  Serial.print(detail_valid ? F("OK") : F("LOST"));
  Serial.print(F(" part1="));
  Serial.print(part1_valid ? F("VALID") : F("INVALID"));
  Serial.print(F(" part2="));
  Serial.println(part2_valid ? F("VALID") : F("INVALID"));

  Serial.print(F("  GNSS state="));
  Serial.println(gnss_state_text(values.gnss_state));

  Serial.print(F("  MOTION mode="));
  if (!vehicle_valid)
  {
    Serial.print(F("-- gear=-- speed=--"));
  }
  else
  {
    Serial.print(flag_is_set(values.status_flags, STATUS_MODE_AUTO)
                     ? F("AUTO") : F("MANUAL"));
    Serial.print(F(" gear="));
    Serial.print(flag_is_set(values.status_flags, STATUS_GEAR_REVERSE)
                     ? F("REV") : F("FWD"));
    Serial.print(F(" speed="));
    print_hundredths(values.speed_centi_mps);
    Serial.print(F(" m/s display="));
    print_display_kmh(values.speed_centi_mps);
    Serial.print(F(" km/h"));
  }
  Serial.println();

  Serial.print(F("  SAFETY estop="));
  Serial.print(yes_no(flag_is_set(values.status_flags, STATUS_ESTOP_ACTIVE)));
  Serial.print(F(" brake="));
  Serial.println(yes_no(flag_is_set(values.status_flags, STATUS_BRAKE_ACTIVE)));

  Serial.print(F("  DRIVE pedal="));
  if (!detail_valid)
  {
    Serial.println(F("-- tractive=-- command=-- drive_stop=--"));
  }
  else
  {
    Serial.print(values.pedal_percent);
    Serial.print(F("% tractive="));
    Serial.print(flag_is_set(values.drive_flags, DRIVE_TRACTIVE_FEEDBACK)
                     ? F("RUN") : F("STOP"));
    Serial.print(F(" command="));
    if (flag_is_set(values.drive_flags, DRIVE_COMMAND_TIMEOUT))
    {
      Serial.print(F("TIMEOUT"));
    }
    else
    {
      Serial.print(flag_is_set(values.drive_flags, DRIVE_COMMAND_RECEIVED)
                       ? F("OK") : F("WAIT"));
    }
    Serial.print(F(" drive_stop="));
    Serial.println(yes_no(flag_is_set(values.drive_flags, DRIVE_STOP)));
  }

  Serial.print(F("  POWER voltage="));
  if (!detail_valid || !part1_valid)
  {
    Serial.println(F("-- bus_current=-- phase_current=--"));
  }
  else
  {
    Serial.print(values.bus_voltage_decivolt / 10U);
    Serial.print('.');
    Serial.print(values.bus_voltage_decivolt % 10U);
    Serial.print(F(" V bus_current="));
    print_tenths(values.bus_current_deciamp);
    Serial.print(F(" A phase_current="));
    print_tenths(values.phase_current_deciamp);
    Serial.println(F(" A"));
  }

  Serial.print(F("  THERMAL controller="));
  if (!detail_valid || !part2_valid)
  {
    Serial.println(F("-- motor=-- fault=--"));
  }
  else
  {
    print_tenths(values.controller_temperature_decic);
    Serial.print(F(" C motor="));
    print_tenths(values.motor_temperature_decic);
    Serial.print(F(" C fault=0x"));
    print_hex24(values.motor_fault_bits);
    Serial.println();
  }
}

static void start_snapshot(uint8_t alive)
{
  assembly.values = {};
  assembly.alive = alive;
  assembly.received_frames = 0U;
  assembly.active = true;
}

static bool is_dashboard_id(uint32_t identifier)
{
  return (identifier == DASHBOARD_STATUS_ID) ||
         (identifier == DASHBOARD_POWER_ID) ||
         (identifier == DASHBOARD_THERMAL_ID) ||
         (identifier == DASHBOARD_FAULT_ID);
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
        Serial.println(F("CAN RECOVERED"));
        can_lost = false;
      }
      DashboardUi::update(latest_values, false);
      if (!printed_snapshot ||
          ((uint32_t)(now - last_serial_print_ms) >= SERIAL_PRINT_PERIOD_MS))
      {
        print_snapshot(assembly.values, assembly.alive);
        last_serial_print_ms = now;
        printed_snapshot = true;
      }
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

static void try_reinitialize_can()
{
  if (can_controller_ready)
  {
    drain_can_receive_buffer();
  }
  CAN.end();
  can_controller_ready = false;

  assembly.active = false;
  if (CAN.begin(CanBitRate::BR_250k))
  {
    can_controller_ready = true;
    has_completed_alive = false;
    can_reinit_count++;
    drain_can_receive_buffer();
    Serial.println(F("CAN controller reinitialized; waiting for fresh alive"));
  }
  else
  {
    Serial.println(F("CAN controller reinitialize failed"));
  }
}

void setup()
{
  Serial.begin(115200);

  St7796Lcd::begin();
  St7796Lcd::setBacklight(true);
  ManualTftUi::playStartup();
  can_controller_ready = CAN.begin(CanBitRate::BR_250k);
  DashboardUi::begin(latest_values, true);
  Serial.println(F("LCD dashboard UI rendered"));

  if (!can_controller_ready)
  {
    Serial.println(F("CAN.begin(BR_250k) failed"));
  }

  last_complete_snapshot_ms = millis();
  last_can_reinit_ms = last_complete_snapshot_ms;
  Serial.println(F("UNO R4 dashboard CAN protocol v1 receiver ready"));
  Serial.println(F("Waiting for standard IDs 0x321-0x324, DLC=8"));
}

void loop()
{
  while (can_controller_ready && (CAN.available() > 0U))
  {
    accept_dashboard_frame(CAN.read());
  }

  const uint32_t now = millis();
  if (!can_lost &&
      ((uint32_t)(now - last_complete_snapshot_ms) >= CAN_LOST_TIMEOUT_MS))
  {
    Serial.println(F("CAN LOST: no fresh complete snapshot for 300 ms"));
    can_lost = true;
    assembly.active = false;
    DashboardUi::update(latest_values, true);
  }

  if (can_lost &&
      ((uint32_t)(now - last_can_reinit_ms) >= CAN_REINIT_PERIOD_MS))
  {
    last_can_reinit_ms = now;
    try_reinitialize_can();
  }
}
