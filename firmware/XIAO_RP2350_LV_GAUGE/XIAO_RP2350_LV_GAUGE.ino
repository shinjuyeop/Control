#include <Arduino.h>
#include <LittleFS.h>
#include <Wire.h>

#include <limits.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace {

// XIAO RP2350 D4/D5 are GPIO6/GPIO7 on the I2C1 peripheral.
constexpr uint8_t kSdaPin = 6;
constexpr uint8_t kSclPin = 7;
constexpr uint32_t kI2cClockHz = 400000;

// INA228 supports 16 address combinations selected by A1/A0.
constexpr uint8_t kFirstInaAddress = 0x40;
constexpr uint8_t kLastInaAddress = 0x4F;

constexpr uint8_t kConfigRegister = 0x00;
constexpr uint8_t kAdcConfigRegister = 0x01;
constexpr uint8_t kShuntCalRegister = 0x02;
constexpr uint8_t kVshuntRegister = 0x04;
constexpr uint8_t kVbusRegister = 0x05;
constexpr uint8_t kDieTempRegister = 0x06;
constexpr uint8_t kCurrentRegister = 0x07;
constexpr uint8_t kChargeRegister = 0x0A;
constexpr uint8_t kDiagAlertRegister = 0x0B;
constexpr uint8_t kManufacturerIdRegister = 0x3E;
constexpr uint8_t kDeviceIdRegister = 0x3F;

constexpr uint16_t kManufacturerId = 0x5449;  // TI in ASCII.
constexpr uint16_t kIna228DeviceId = 0x2281;  // DIEID=0x228, REV_ID=1.

// FL-2C: 100 A / 75 mV = 0.00075 ohm.
constexpr float kShuntResistanceOhm = 0.00075f;
constexpr float kCurrentLsbA = 0.0002f;  // 0.2 mA/bit, about 105 A full scale.
constexpr int64_t kCurrentLsbUa = 200;
// 13107.2e6 * CURRENT_LSB * RSHUNT, rounded for ADCRANGE=0.
constexpr uint16_t kShuntCalibration = 1966;

// Continuous VBUS + VSHUNT + DIETEMP: 50 us each, one sample (150 us cycle).
// ADCRANGE=0 selects +/-163.84 mV, which covers the 75 mV shunt at 100 A.
constexpr uint16_t kAdcConfig = 0xF000;
constexpr uint16_t kExpectedConfig = 0x0000;
constexpr float kShuntVoltageLsbV = 312.5e-9f;
constexpr float kBusVoltageLsbV = 195.3125e-6f;
constexpr float kDieTemperatureLsbMilliC = 7.8125f;
constexpr uint16_t kConversionReadyFlag = 1U << 1;
constexpr uint16_t kMathOverflowFlag = 1U << 9;
constexpr uint16_t kAccumulatorFaultMask = (1U << 11) | (1U << 10) | (1U << 9);
constexpr uint64_t kAccumulatorMask40 = (1ULL << 40) - 1ULL;
constexpr uint64_t kAccumulatorSign40 = 1ULL << 39;

constexpr float kFilterTimeConstantUs = 100000.0f;  // 100 ms, about 1.59 Hz cutoff.
constexpr uint32_t kProbePeriodMs = 1000;
constexpr uint32_t kStatusPeriodMs = 100;
constexpr uint32_t kPersistPeriodMs = 60000;
constexpr uint32_t kConversionStaleTimeoutMs = 250;
constexpr uint32_t kConfigurationCheckPeriodMs = 100;
constexpr uint32_t kZeroSampleCount = 64;
constexpr uint32_t kZeroSampleWaitTimeoutMs = 500;

constexpr uint64_t kMicroampSecondsPerMilliampHour = 3600000ULL;

enum : uint8_t {
  kMessageStatus = 0x01,
  kMessageCommand = 0x10,
  kMessageCommandAck = 0x11,
};

enum : uint8_t {
  kCommandSetFull = 0x01,
  kCommandResetTrip = 0x02,
  kCommandZeroCurrent = 0x03,
  kCommandSetCapacity = 0x04,
};

enum : uint8_t {
  kResultSuccess = 0x00,
  kResultInvalid = 0x01,
  kResultStorageFailed = 0x02,
  kResultSensorUnavailable = 0x03,
  kResultBusy = 0x04,
  kResultRequestConflict = 0x05,
};

enum : uint8_t {
  kStatusSensorIdentified = 1U << 0,
  kStatusMeasurementValid = 1U << 1,
  kStatusSocKnown = 1U << 2,
  kStatusStorageMounted = 1U << 3,
  kStatusZeroCalibrating = 1U << 4,
  kStatusStorageDirty = 1U << 5,
  kStatusCommandBusy = 1U << 6,
  kStatusProtocolReady = 1U << 7,
};

enum : uint8_t {
  kSensorNotDetected = 0,
  kSensorReady = 1,
  kSensorReadError = 2,
  kSensorConfiguring = 3,
};

struct Reading {
  uint32_t bus_counts;
  int32_t shunt_counts;
  int32_t current_counts;
  uint16_t config_register;
  uint16_t adc_config_register;
  uint16_t shunt_cal_register;
  uint16_t diag_alert;
  int16_t die_temperature_counts;
  uint32_t vbus_raw24;
  uint32_t vshunt_raw24;
  uint32_t current_raw24;
  uint64_t charge_raw;
  float bus_voltage_v;
  float shunt_voltage_v;
  float current_a;
};

struct __attribute__((packed)) PersistentState {
  uint32_t magic;
  uint16_t version;
  uint16_t bytes;
  uint32_t sequence;
  uint8_t soc_known;
  uint8_t reserved0[3];
  uint32_t capacity_mAh;
  int32_t current_offset_counts;
  int64_t remaining_uAs;
  uint64_t charge_trip_uAs;
  uint64_t discharge_trip_uAs;
  uint64_t charge_trip_mWs;
  uint64_t discharge_trip_mWs;
  uint64_t active_time_ms;
  uint32_t last_request_id;
  uint8_t last_command;
  uint8_t last_result;
  uint16_t reserved1;
  uint32_t crc32;
};

struct __attribute__((packed)) CommandPayload {
  uint32_t request_id;
  uint8_t command;
  uint8_t reserved[3];
  int32_t value;
};

struct __attribute__((packed)) AckPayload {
  uint32_t request_id;
  uint8_t command;
  uint8_t result;
  uint16_t reserved;
  uint32_t storage_sequence;
  int32_t detail;
};

struct __attribute__((packed)) StatusPayload {
  uint32_t uptime_ms;
  uint8_t flags;
  uint8_t sensor_state;
  uint8_t ina_address;
  uint8_t reserved;
  uint16_t manufacturer_id;
  uint16_t device_id;
  uint16_t config_register;
  uint16_t adc_config_register;
  uint16_t shunt_cal_register;
  uint16_t diag_alert;
  uint32_t vbus_raw24;
  uint32_t vshunt_raw24;
  uint32_t current_raw24;
  int32_t bus_voltage_uV;
  int32_t shunt_voltage_nV;
  int32_t current_uA;
  int32_t power_mW;
  int32_t current_offset_uA;
  int32_t die_temperature_mC;
  int32_t soc_centi_percent;
  int64_t remaining_uAs;
  uint32_t capacity_mAh;
  uint64_t charge_trip_uAs;
  uint64_t discharge_trip_uAs;
  uint64_t charge_trip_mWs;
  uint64_t discharge_trip_mWs;
  uint64_t active_time_ms;
  uint32_t storage_sequence;
  uint32_t i2c_error_count;
  uint32_t storage_error_count;
  uint32_t sample_sequence;
  uint32_t last_valid_age_ms;
  uint32_t accumulator_fault_count;
};

static_assert(sizeof(PersistentState) == 84, "persistent state layout changed");
static_assert(sizeof(CommandPayload) == 12, "command payload layout changed");
static_assert(sizeof(AckPayload) == 16, "ack payload layout changed");
static_assert(sizeof(StatusPayload) == 136, "status payload layout changed");

constexpr uint32_t kPersistentMagic = 0x4C564731;  // LVG1.
constexpr uint16_t kPersistentVersion = 1;
constexpr uint16_t kJournalRecordsPerFile = 512;
constexpr size_t kProtocolHeaderBytes = 10;
constexpr size_t kProtocolCrcBytes = 4;
constexpr size_t kProtocolMaxPayload = sizeof(StatusPayload);
constexpr size_t kProtocolMaxFrame =
    kProtocolHeaderBytes + kProtocolMaxPayload + kProtocolCrcBytes;
constexpr char kJournalPaths[2][20] = {"/lvg_state_a.bin", "/lvg_state_b.bin"};

void putU16Le(uint8_t* destination, uint16_t value) {
  destination[0] = static_cast<uint8_t>(value & 0xFFU);
  destination[1] = static_cast<uint8_t>(value >> 8);
}

void putU32Le(uint8_t* destination, uint32_t value) {
  destination[0] = static_cast<uint8_t>(value & 0xFFU);
  destination[1] = static_cast<uint8_t>((value >> 8) & 0xFFU);
  destination[2] = static_cast<uint8_t>((value >> 16) & 0xFFU);
  destination[3] = static_cast<uint8_t>(value >> 24);
}

uint16_t getU16Le(const uint8_t* source) {
  return static_cast<uint16_t>(source[0]) |
         (static_cast<uint16_t>(source[1]) << 8);
}

uint32_t getU32Le(const uint8_t* source) {
  return static_cast<uint32_t>(source[0]) |
         (static_cast<uint32_t>(source[1]) << 8) |
         (static_cast<uint32_t>(source[2]) << 16) |
         (static_cast<uint32_t>(source[3]) << 24);
}

uint32_t crc32(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFFU;
  for (size_t index = 0; index < length; ++index) {
    crc ^= data[index];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) != 0U ? (crc >> 1) ^ 0xEDB88320U : crc >> 1;
    }
  }
  return ~crc;
}

int64_t roundToInt64(double value) {
  return value >= 0.0 ? static_cast<int64_t>(value + 0.5)
                     : static_cast<int64_t>(value - 0.5);
}

bool sequenceIsNewer(uint32_t candidate, uint32_t current) {
  return static_cast<int32_t>(candidate - current) > 0;
}

bool stateIsValid(const PersistentState& state) {
  if (state.magic != kPersistentMagic || state.version != kPersistentVersion ||
      state.bytes != sizeof(PersistentState) || state.capacity_mAh == 0U ||
      state.soc_known > 1U) {
    return false;
  }
  if ((state.soc_known == 0U && state.remaining_uAs != -1) ||
      (state.soc_known != 0U && state.remaining_uAs < 0)) {
    return false;
  }
  return crc32(reinterpret_cast<const uint8_t*>(&state),
               offsetof(PersistentState, crc32)) == state.crc32;
}

void initializeDefaultState(PersistentState& state) {
  memset(&state, 0, sizeof(state));
  state.magic = kPersistentMagic;
  state.version = kPersistentVersion;
  state.bytes = sizeof(PersistentState);
  state.capacity_mAh = 100000U;  // 100 Ah.
  state.remaining_uAs = -1;
}

class Ina228 {
 public:
  Ina228(TwoWire& bus, uint8_t address) : bus_(bus), address_(address) {}

  void setAddress(uint8_t address) { address_ = address; }

  bool read16(uint8_t reg, uint16_t& value) {
    if (!selectRegister(reg) || bus_.requestFrom(address_, 2, true) != 2) {
      return false;
    }
    const int first = bus_.read();
    const int second = bus_.read();
    if (first < 0 || second < 0) {
      return false;
    }
    value = (static_cast<uint16_t>(first) << 8) |
            static_cast<uint16_t>(second);
    return true;
  }

  bool read24(uint8_t reg, uint32_t& value) {
    if (!selectRegister(reg) || bus_.requestFrom(address_, 3, true) != 3) {
      return false;
    }
    const int first = bus_.read();
    const int second = bus_.read();
    const int third = bus_.read();
    if (first < 0 || second < 0 || third < 0) {
      return false;
    }
    value = (static_cast<uint32_t>(first) << 16) |
            (static_cast<uint32_t>(second) << 8) |
            static_cast<uint32_t>(third);
    return true;
  }

  bool read40(uint8_t reg, uint64_t& value) {
    if (!selectRegister(reg) || bus_.requestFrom(address_, 5, true) != 5) {
      return false;
    }
    uint64_t assembled = 0;
    for (uint8_t index = 0; index < 5; ++index) {
      const int byte_value = bus_.read();
      if (byte_value < 0) {
        return false;
      }
      assembled = (assembled << 8) | static_cast<uint8_t>(byte_value);
    }
    value = assembled & kAccumulatorMask40;
    return true;
  }

  bool write16(uint8_t reg, uint16_t value) {
    bus_.beginTransmission(address_);
    bus_.write(reg);
    bus_.write(static_cast<uint8_t>(value >> 8));
    bus_.write(static_cast<uint8_t>(value & 0xFFU));
    return bus_.endTransmission(true) == 0;
  }

  bool configure() {
    // CONFIG.RST restores all INA228 registers, including CHARGE/ENERGY.
    if (!write16(kConfigRegister, 0x8000U)) {
      return false;
    }
    delay(2);
    if (!write16(kAdcConfigRegister, kAdcConfig) ||
        !write16(kShuntCalRegister, kShuntCalibration)) {
      return false;
    }
    uint16_t calibration_readback = 0;
    return read16(kShuntCalRegister, calibration_readback) &&
           calibration_readback == kShuntCalibration;
  }

  bool readConfiguration(uint16_t& config_register,
                         uint16_t& adc_config_register,
                         uint16_t& shunt_cal_register) {
    return read16(kConfigRegister, config_register) &&
           read16(kAdcConfigRegister, adc_config_register) &&
           read16(kShuntCalRegister, shunt_cal_register);
  }

  bool readConversion(Reading& reading, bool& conversion_ready) {
    uint32_t bus_raw = 0;
    uint32_t shunt_raw = 0;
    uint32_t current_raw = 0;
    uint16_t die_temperature_raw = 0;
    uint16_t diag_alert = 0;
    conversion_ready = false;
    // Reading DIAG_ALRT clears CNVRF, so it must be read once before data.
    if (!read16(kDiagAlertRegister, diag_alert)) {
      return false;
    }
    reading.diag_alert = diag_alert;
    conversion_ready = (diag_alert & kConversionReadyFlag) != 0U;
    if (!conversion_ready) {
      return true;
    }
    if (!read16(kConfigRegister, reading.config_register) ||
        !read16(kAdcConfigRegister, reading.adc_config_register) ||
        !read16(kShuntCalRegister, reading.shunt_cal_register) ||
        !read16(kDieTempRegister, die_temperature_raw) ||
        !read24(kVbusRegister, bus_raw) ||
        !read24(kVshuntRegister, shunt_raw) ||
        !read24(kCurrentRegister, current_raw) ||
        !read40(kChargeRegister, reading.charge_raw)) {
      return false;
    }

    reading.bus_counts = bus_raw >> 4;
    reading.shunt_counts = signExtend20(shunt_raw >> 4);
    reading.current_counts = signExtend20(current_raw >> 4);
    reading.vbus_raw24 = bus_raw;
    reading.vshunt_raw24 = shunt_raw;
    reading.current_raw24 = current_raw;
    reading.die_temperature_counts = static_cast<int16_t>(die_temperature_raw);
    reading.bus_voltage_v = static_cast<float>(reading.bus_counts) * kBusVoltageLsbV;
    reading.shunt_voltage_v =
        static_cast<float>(reading.shunt_counts) * kShuntVoltageLsbV;
    reading.current_a = static_cast<float>(reading.current_counts) * kCurrentLsbA;
    return true;
  }

 private:
  bool selectRegister(uint8_t reg) {
    bus_.beginTransmission(address_);
    bus_.write(reg);
    return bus_.endTransmission(false) == 0;
  }

  static int32_t signExtend20(uint32_t value) {
    value &= 0x000FFFFFU;
    if ((value & 0x00080000U) != 0U) {
      value |= 0xFFF00000U;
    }
    return static_cast<int32_t>(value);
  }

  TwoWire& bus_;
  uint8_t address_;
};

  Ina228 ina228(Wire1, kFirstInaAddress);
PersistentState gauge_state;

bool storage_mounted = false;
bool journal_has_state = false;
uint8_t journal_active_slot = 0;
uint16_t journal_active_records = 0;
bool journal_active_tail_clean = true;
uint32_t last_persist_attempt_ms = 0;
bool state_dirty = false;

bool sensor_identified = false;
bool sensor_ready = false;
uint8_t sensor_state = kSensorNotDetected;
uint8_t ina_address = 0;
uint16_t ina_manufacturer_id = 0;
uint16_t ina_device_id = 0;
uint16_t ina_config_register = 0;
uint16_t ina_adc_config_register = 0;
uint16_t ina_shunt_cal_register = 0;
uint16_t ina_diag_alert = 0;
uint32_t i2c_error_count = 0;
uint32_t storage_error_count = 0;
uint32_t accumulator_fault_count = 0;
uint32_t sample_sequence = 0;
uint32_t last_status_ms = 0;
uint32_t last_filter_us = 0;
float filtered_bus_v = 0.0f;
float filtered_shunt_v = 0.0f;
float filtered_current_a = 0.0f;
uint32_t last_conversion_ms = 0;
uint32_t last_configuration_check_ms = 0;
uint32_t last_probe_ms = 0;
uint32_t last_valid_sample_ms = 0;
bool measurement_valid = false;
bool have_valid_sample = false;
bool zero_calibrating = false;
bool command_busy = false;
bool have_accumulator_baseline = false;
uint64_t previous_charge_raw = 0;
float previous_charge_voltage_v = 0.0f;
double charge_energy_remainder_mWs = 0.0;
double discharge_energy_remainder_mWs = 0.0;
uint16_t previous_accumulator_faults = 0;
Reading last_reading{};

uint8_t usb_rx_buffer[kProtocolMaxFrame * 2];
size_t usb_rx_length = 0;
uint32_t tx_sequence = 0;

bool readJournalRecord(File& file, PersistentState& state) {
  if (file.read(reinterpret_cast<uint8_t*>(&state), sizeof(state)) !=
      static_cast<int>(sizeof(state))) {
    return false;
  }
  return stateIsValid(state);
}

void loadPersistentState() {
  initializeDefaultState(gauge_state);
  if (!LittleFS.begin()) {
    storage_error_count++;
    return;
  }
  storage_mounted = true;

  bool found = false;
  bool any_file = false;
  PersistentState newest{};
  for (uint8_t slot = 0; slot < 2; ++slot) {
    File file = LittleFS.open(kJournalPaths[slot], "r");
    if (!file) {
      continue;
    }
    any_file = true;
    const size_t physical_bytes = file.size();
    const size_t record_count =
        (physical_bytes / sizeof(PersistentState)) > kJournalRecordsPerFile
            ? kJournalRecordsPerFile
            : physical_bytes / sizeof(PersistentState);
    for (size_t index = 0; index < record_count; ++index) {
      PersistentState candidate{};
      if (!readJournalRecord(file, candidate)) {
        break;
      }
      if (!found || sequenceIsNewer(candidate.sequence, newest.sequence)) {
        newest = candidate;
        found = true;
        journal_active_slot = slot;
        journal_active_records = static_cast<uint16_t>(index + 1U);
        journal_active_tail_clean =
            physical_bytes == (index + 1U) * sizeof(PersistentState);
      }
    }
    file.close();
  }

  if (found) {
    gauge_state = newest;
    journal_has_state = true;
  } else if (any_file) {
    // Preserve the option of writing the other journal file first.
    journal_active_records = kJournalRecordsPerFile;
    journal_active_tail_clean = false;
  }
}

bool verifyJournalRecord(const char* path, uint32_t offset,
                         const PersistentState& expected) {
  File file = LittleFS.open(path, "r");
  if (!file || file.size() < offset + sizeof(PersistentState) ||
      !file.seek(offset, SeekSet)) {
    if (file) {
      file.close();
    }
    return false;
  }
  PersistentState actual{};
  const bool read_ok = readJournalRecord(file, actual);
  file.close();
  return read_ok && memcmp(&actual, &expected, sizeof(expected)) == 0;
}

bool writePersistentState(const PersistentState& requested, uint32_t now_ms) {
  if (!storage_mounted) {
    return false;
  }

  PersistentState next = requested;
  next.magic = kPersistentMagic;
  next.version = kPersistentVersion;
  next.bytes = sizeof(PersistentState);
  next.sequence = journal_has_state ? gauge_state.sequence + 1U : 1U;
  next.crc32 = crc32(reinterpret_cast<const uint8_t*>(&next),
                     offsetof(PersistentState, crc32));

  const bool replace_file = !journal_has_state ||
                            journal_active_records >= kJournalRecordsPerFile ||
                            !journal_active_tail_clean;
  const uint8_t target_slot =
      replace_file ? static_cast<uint8_t>(1U - journal_active_slot)
                   : journal_active_slot;
  const uint32_t offset = replace_file
                              ? 0U
                              : static_cast<uint32_t>(journal_active_records) *
                                    sizeof(PersistentState);
  File file = LittleFS.open(kJournalPaths[target_slot], replace_file ? "w" : "a");
  if (!file) {
    journal_active_tail_clean = false;
    return false;
  }
  const size_t written = file.write(reinterpret_cast<const uint8_t*>(&next),
                                    sizeof(next));
  file.flush();
  file.close();
  if (written != sizeof(next) ||
      !verifyJournalRecord(kJournalPaths[target_slot], offset, next)) {
    journal_active_tail_clean = false;
    return false;
  }

  gauge_state = next;
  journal_has_state = true;
  journal_active_slot = target_slot;
  journal_active_records = replace_file ? 1U : journal_active_records + 1U;
  journal_active_tail_clean = true;
  state_dirty = false;
  last_persist_attempt_ms = now_ms;
  return true;
}

int32_t socCentiPercent() {
  if (gauge_state.soc_known == 0U || gauge_state.capacity_mAh == 0U ||
      gauge_state.remaining_uAs < 0) {
    return -1;
  }
  const uint64_t capacity_uAs =
      static_cast<uint64_t>(gauge_state.capacity_mAh) *
      kMicroampSecondsPerMilliampHour;
  uint64_t remaining_uAs = static_cast<uint64_t>(gauge_state.remaining_uAs);
  if (remaining_uAs > capacity_uAs) {
    remaining_uAs = capacity_uAs;
  }
  const uint64_t percent = (remaining_uAs * 10000ULL) / capacity_uAs;
  return static_cast<int32_t>(percent > 10000ULL ? 10000ULL : percent);
}

uint64_t saturatingAdd(uint64_t destination, uint64_t value) {
  if (UINT64_MAX - destination < value) {
    return UINT64_MAX;
  }
  return destination + value;
}

int64_t signedAccumulatorDelta(uint64_t newer, uint64_t older) {
  const uint64_t difference = (newer - older) & kAccumulatorMask40;
  if ((difference & kAccumulatorSign40) != 0U) {
    return static_cast<int64_t>(difference) - static_cast<int64_t>(1ULL << 40);
  }
  return static_cast<int64_t>(difference);
}

void observeAccumulatorFlags(uint16_t diag_alert) {
  const uint16_t accumulator_faults = diag_alert & kAccumulatorFaultMask;
  if ((accumulator_faults & static_cast<uint16_t>(~previous_accumulator_faults)) != 0U) {
    accumulator_fault_count++;
  }
  previous_accumulator_faults = accumulator_faults;
}

void integrateConversion(const Reading& reading, uint32_t now_ms) {
  observeAccumulatorFlags(reading.diag_alert);
  if (!have_accumulator_baseline) {
    previous_charge_raw = reading.charge_raw;
    previous_charge_voltage_v = reading.bus_voltage_v;
    previous_accumulator_faults = reading.diag_alert & kAccumulatorFaultMask;
    last_conversion_ms = now_ms;
    have_accumulator_baseline = true;
    return;
  }

  const uint32_t dt_ms = now_ms - last_conversion_ms;
  // Keep the previous accumulator baseline until a full millisecond elapsed.
  if (dt_ms == 0U) {
    return;
  }
  last_conversion_ms = now_ms;
  const int64_t charge_counts_delta =
      signedAccumulatorDelta(reading.charge_raw, previous_charge_raw);
  const double interval_voltage_v =
      (static_cast<double>(previous_charge_voltage_v) + reading.bus_voltage_v) * 0.5;
  previous_charge_raw = reading.charge_raw;
  previous_charge_voltage_v = reading.bus_voltage_v;

  if (dt_ms > kConversionStaleTimeoutMs) {
    return;
  }

  // CHARGE is the signed time integral from INA228. Correct its result for
  // the software zero offset, which INA228 itself cannot store.
  const int64_t raw_delta_uAs = charge_counts_delta * kCurrentLsbUa;
  const int64_t offset_delta_uAs = roundToInt64(
      static_cast<double>(gauge_state.current_offset_counts) *
      kCurrentLsbUa * dt_ms / 1000.0);
  const int64_t delta_uAs = raw_delta_uAs - offset_delta_uAs;

  gauge_state.active_time_ms += dt_ms;
  // Both trip units describe the same net charge interval. Voltage is the
  // mean of its endpoints; uA*s * V / 1000 gives mW*s. Carry sub-mWs parts
  // separately by direction so small currents survive frequent updates.
  const double energy_mWs =
      fabs(static_cast<double>(delta_uAs)) * interval_voltage_v / 1000.0;
  if (delta_uAs > 0) {
    gauge_state.discharge_trip_uAs =
        saturatingAdd(gauge_state.discharge_trip_uAs,
                      static_cast<uint64_t>(delta_uAs));
    discharge_energy_remainder_mWs += energy_mWs;
    const uint64_t whole_mWs = static_cast<uint64_t>(discharge_energy_remainder_mWs);
    discharge_energy_remainder_mWs -= whole_mWs;
    gauge_state.discharge_trip_mWs =
        saturatingAdd(gauge_state.discharge_trip_mWs, whole_mWs);
  } else if (delta_uAs < 0) {
    gauge_state.charge_trip_uAs =
        saturatingAdd(gauge_state.charge_trip_uAs,
                      static_cast<uint64_t>(-delta_uAs));
    charge_energy_remainder_mWs += energy_mWs;
    const uint64_t whole_mWs = static_cast<uint64_t>(charge_energy_remainder_mWs);
    charge_energy_remainder_mWs -= whole_mWs;
    gauge_state.charge_trip_mWs =
        saturatingAdd(gauge_state.charge_trip_mWs, whole_mWs);
  }

  if (gauge_state.soc_known != 0U) {
    gauge_state.remaining_uAs -= delta_uAs;
    const int64_t capacity_uAs = static_cast<int64_t>(
        static_cast<uint64_t>(gauge_state.capacity_mAh) *
        kMicroampSecondsPerMilliampHour);
    if (gauge_state.remaining_uAs < 0) {
      gauge_state.remaining_uAs = 0;
    } else if (gauge_state.remaining_uAs > capacity_uAs) {
      gauge_state.remaining_uAs = capacity_uAs;
    }
  }
  state_dirty = true;
}

bool findIna228(uint8_t& address, uint16_t& manufacturer_id,
                uint16_t& device_id) {
  for (uint8_t candidate_address = kFirstInaAddress;
       candidate_address <= kLastInaAddress; ++candidate_address) {
    Ina228 candidate(Wire1, candidate_address);
    uint16_t candidate_manufacturer = 0;
    uint16_t candidate_device = 0;
    if (!candidate.read16(kManufacturerIdRegister, candidate_manufacturer) ||
        !candidate.read16(kDeviceIdRegister, candidate_device)) {
      continue;
    }
    if (candidate_manufacturer == kManufacturerId &&
        (candidate_device >> 4) == (kIna228DeviceId >> 4)) {
      address = candidate_address;
      manufacturer_id = candidate_manufacturer;
      device_id = candidate_device;
      return true;
    }
  }
  return false;
}

bool connectIna228() {
  sensor_state = kSensorConfiguring;
  uint8_t address = 0;
  uint16_t manufacturer_id = 0;
  uint16_t device_id = 0;
  if (!findIna228(address, manufacturer_id, device_id)) {
    sensor_identified = false;
    sensor_ready = false;
    sensor_state = kSensorNotDetected;
    return false;
  }

  ina_address = address;
  ina_manufacturer_id = manufacturer_id;
  ina_device_id = device_id;
  sensor_identified = true;
  ina228.setAddress(address);
  if (!ina228.configure()) {
    sensor_ready = false;
    sensor_state = kSensorConfiguring;
    i2c_error_count++;
    return false;
  }
  if (!ina228.readConfiguration(ina_config_register, ina_adc_config_register,
                                ina_shunt_cal_register) ||
      ina_config_register != kExpectedConfig ||
      ina_adc_config_register != kAdcConfig ||
      ina_shunt_cal_register != kShuntCalibration) {
    sensor_ready = false;
    sensor_state = kSensorConfiguring;
    i2c_error_count++;
    return false;
  }
  sensor_ready = true;
  sensor_state = kSensorReady;
  measurement_valid = false;
  have_accumulator_baseline = false;
  previous_accumulator_faults = 0;
  const uint32_t now_ms = millis();
  last_conversion_ms = now_ms;
  last_configuration_check_ms = now_ms;
  return true;
}

bool sendFrame(uint8_t message_type, const void* payload, size_t payload_size);
void sendStatus();

bool collectZeroOffset(int32_t& offset_counts) {
  if (!sensor_ready) {
    return false;
  }
  zero_calibrating = true;
  measurement_valid = false;
  have_accumulator_baseline = false;
  sendStatus();

  int64_t sum = 0;
  for (uint32_t index = 0; index < kZeroSampleCount; ++index) {
    const uint32_t wait_start_ms = millis();
    bool conversion_ready = false;
    Reading sample{};
    for (;;) {
      if (!ina228.readConversion(sample, conversion_ready)) {
        i2c_error_count++;
        sensor_ready = false;
        sensor_state = kSensorReadError;
        zero_calibrating = false;
        return false;
      }
      if (conversion_ready) {
        break;
      }
      if (millis() - wait_start_ms >= kZeroSampleWaitTimeoutMs) {
        sensor_ready = false;
        sensor_state = kSensorReadError;
        zero_calibrating = false;
        return false;
      }
      delay(1);
    }
    if (sample.config_register != kExpectedConfig ||
        sample.adc_config_register != kAdcConfig ||
        sample.shunt_cal_register != kShuntCalibration) {
      sensor_ready = false;
      sensor_state = kSensorConfiguring;
      zero_calibrating = false;
      return false;
    }
    ina_config_register = sample.config_register;
    ina_adc_config_register = sample.adc_config_register;
    ina_shunt_cal_register = sample.shunt_cal_register;
    ina_diag_alert = sample.diag_alert;
    sum += sample.current_counts;
  }
  offset_counts = static_cast<int32_t>(
      sum >= 0 ? (sum + kZeroSampleCount / 2U) / kZeroSampleCount
               : (sum - kZeroSampleCount / 2U) / kZeroSampleCount);
  zero_calibrating = false;
  return true;
}

void sendAck(uint32_t request_id, uint8_t command, uint8_t result,
             int32_t detail) {
  AckPayload ack{};
  ack.request_id = request_id;
  ack.command = command;
  ack.result = result;
  ack.storage_sequence = gauge_state.sequence;
  ack.detail = detail;
  sendFrame(kMessageCommandAck, &ack, sizeof(ack));
}

void handleCommand(const CommandPayload& command) {
  if (command.request_id == 0U) {
    sendAck(command.request_id, command.command, kResultInvalid, 0);
    return;
  }

  // The request id is persisted with the mutation. If an ACK was lost during
  // USB reconnect, replaying the same request returns the original result and
  // cannot clear a newly accumulated trip a second time.
  if (journal_has_state && command.request_id == gauge_state.last_request_id) {
    if (command.command == gauge_state.last_command) {
      sendAck(command.request_id, command.command, gauge_state.last_result, 0);
    } else {
      sendAck(command.request_id, command.command, kResultRequestConflict, 0);
    }
    return;
  }
  if (command_busy) {
    sendAck(command.request_id, command.command, kResultBusy, 0);
    return;
  }

  PersistentState candidate = gauge_state;
  int32_t detail = 0;
  bool command_valid = true;
  command_busy = true;

  switch (command.command) {
    case kCommandSetFull:
      candidate.soc_known = 1U;
      candidate.remaining_uAs = static_cast<int64_t>(
          static_cast<uint64_t>(candidate.capacity_mAh) *
          kMicroampSecondsPerMilliampHour);
      detail = static_cast<int32_t>(candidate.capacity_mAh);
      break;

    case kCommandResetTrip:
      candidate.charge_trip_uAs = 0;
      candidate.discharge_trip_uAs = 0;
      candidate.charge_trip_mWs = 0;
      candidate.discharge_trip_mWs = 0;
      candidate.active_time_ms = 0;
      break;

    case kCommandZeroCurrent: {
      int32_t offset_counts = 0;
      if (!collectZeroOffset(offset_counts)) {
        command_busy = false;
        sendAck(command.request_id, command.command,
                kResultSensorUnavailable, 0);
        return;
      }
      candidate.current_offset_counts = offset_counts;
      detail = offset_counts;
      break;
    }

    case kCommandSetCapacity:
      if (command.value <= 0) {
        command_valid = false;
      } else {
        candidate.capacity_mAh = static_cast<uint32_t>(command.value);
        // Capacity is the denominator for SOC. The measured remaining charge
        // is preserved; changing capacity does not silently set SOC or full.
        detail = command.value;
      }
      break;

    default:
      command_valid = false;
      break;
  }

  if (!command_valid) {
    command_busy = false;
    sendAck(command.request_id, command.command, kResultInvalid, 0);
    return;
  }

  candidate.last_request_id = command.request_id;
  candidate.last_command = command.command;
  candidate.last_result = kResultSuccess;
  const uint32_t now_ms = millis();
  const bool persisted = writePersistentState(candidate, now_ms);
  // Do not let the next conversion delta span the command and its flash write.
  // This is also required for the blocking 64-sample zero operation.
  have_accumulator_baseline = false;
  last_conversion_ms = now_ms;
  command_busy = false;
  if (!persisted) {
    storage_error_count++;
    sendAck(command.request_id, command.command, kResultStorageFailed, 0);
    return;
  }
  if (command.command == kCommandResetTrip) {
    charge_energy_remainder_mWs = 0.0;
    discharge_energy_remainder_mWs = 0.0;
  }
  sendAck(command.request_id, command.command, kResultSuccess, detail);
  sendStatus();
}

void consumeUsbFrames() {
  while (usb_rx_length >= kProtocolHeaderBytes) {
    size_t start = 0;
    while (start + 1U < usb_rx_length &&
           !(usb_rx_buffer[start] == 0xA5U &&
             usb_rx_buffer[start + 1U] == 0x5AU)) {
      ++start;
    }
    if (start > 0U) {
      memmove(usb_rx_buffer, usb_rx_buffer + start, usb_rx_length - start);
      usb_rx_length -= start;
    }
    if (usb_rx_length < kProtocolHeaderBytes) {
      return;
    }

    const uint8_t version = usb_rx_buffer[2];
    const uint8_t message_type = usb_rx_buffer[3];
    const uint16_t payload_size = getU16Le(&usb_rx_buffer[4]);
    if (payload_size > kProtocolMaxPayload) {
      memmove(usb_rx_buffer, usb_rx_buffer + 1, --usb_rx_length);
      continue;
    }
    const size_t frame_size =
        kProtocolHeaderBytes + payload_size + kProtocolCrcBytes;
    if (usb_rx_length < frame_size) {
      return;
    }
    const uint32_t received_crc = getU32Le(&usb_rx_buffer[10U + payload_size]);
    const uint32_t expected_crc = crc32(&usb_rx_buffer[2], 8U + payload_size);
    if (version != 1U || received_crc != expected_crc) {
      memmove(usb_rx_buffer, usb_rx_buffer + 1, --usb_rx_length);
      continue;
    }
    if (message_type == kMessageCommand &&
        payload_size == sizeof(CommandPayload)) {
      CommandPayload command{};
      memcpy(&command, &usb_rx_buffer[10], sizeof(command));
      handleCommand(command);
    }
    memmove(usb_rx_buffer, usb_rx_buffer + frame_size,
            usb_rx_length - frame_size);
    usb_rx_length -= frame_size;
  }
}

void processUsb() {
  while (Serial.available() > 0) {
    const int value = Serial.read();
    if (value < 0) {
      break;
    }
    if (usb_rx_length >= sizeof(usb_rx_buffer)) {
      usb_rx_length = 0;
    }
    usb_rx_buffer[usb_rx_length++] = static_cast<uint8_t>(value);
  }
  consumeUsbFrames();
}

bool recheckIna228Configuration(uint32_t now_ms) {
  if (now_ms - last_configuration_check_ms < kConfigurationCheckPeriodMs) {
    return true;
  }
  last_configuration_check_ms = now_ms;
  uint16_t config_register = 0;
  uint16_t adc_config_register = 0;
  uint16_t shunt_cal_register = 0;
  if (!ina228.readConfiguration(config_register, adc_config_register,
                                shunt_cal_register)) {
    i2c_error_count++;
    sensor_ready = false;
    sensor_state = kSensorReadError;
    measurement_valid = false;
    have_accumulator_baseline = false;
    last_probe_ms = now_ms;
    return false;
  }
  if (config_register != kExpectedConfig || adc_config_register != kAdcConfig ||
      shunt_cal_register != kShuntCalibration) {
    measurement_valid = false;
    have_accumulator_baseline = false;
    sensor_ready = false;
    sensor_state = kSensorConfiguring;
    connectIna228();
    return false;
  }
  ina_config_register = config_register;
  ina_adc_config_register = adc_config_register;
  ina_shunt_cal_register = shunt_cal_register;
  return true;
}

void pollConversion(uint32_t now_ms) {
  bool conversion_ready = false;
  Reading raw{};
  if (!ina228.readConversion(raw, conversion_ready)) {
    i2c_error_count++;
    sensor_ready = false;
    sensor_state = kSensorReadError;
    measurement_valid = false;
    have_accumulator_baseline = false;
    last_probe_ms = now_ms;
    return;
  }
  if (!conversion_ready) {
    if (measurement_valid && now_ms - last_conversion_ms > kConversionStaleTimeoutMs) {
      measurement_valid = false;
      have_accumulator_baseline = false;
    }
    return;
  }
  if (raw.config_register != kExpectedConfig ||
      raw.adc_config_register != kAdcConfig ||
      raw.shunt_cal_register != kShuntCalibration) {
    measurement_valid = false;
    have_accumulator_baseline = false;
    sensor_ready = false;
    sensor_state = kSensorConfiguring;
    connectIna228();
    return;
  }
  if ((raw.diag_alert & kMathOverflowFlag) != 0U) {
    observeAccumulatorFlags(raw.diag_alert);
    measurement_valid = false;
    have_accumulator_baseline = false;
    sensor_ready = false;
    sensor_state = kSensorConfiguring;
    connectIna228();
    return;
  }

  raw.current_counts -= gauge_state.current_offset_counts;
  raw.current_a = static_cast<float>(raw.current_counts) * kCurrentLsbA;
  ina_config_register = raw.config_register;
  ina_adc_config_register = raw.adc_config_register;
  ina_shunt_cal_register = raw.shunt_cal_register;
  ina_diag_alert = raw.diag_alert;
  const uint32_t sample_us = micros();
  if (!measurement_valid) {
    filtered_bus_v = raw.bus_voltage_v;
    filtered_shunt_v = raw.shunt_voltage_v;
    filtered_current_a = raw.current_a;
  } else {
    const float dt_us = static_cast<float>(sample_us - last_filter_us);
    const float alpha = -expm1f(-dt_us / kFilterTimeConstantUs);
    filtered_bus_v += alpha * (raw.bus_voltage_v - filtered_bus_v);
    filtered_shunt_v += alpha * (raw.shunt_voltage_v - filtered_shunt_v);
    filtered_current_a += alpha * (raw.current_a - filtered_current_a);
  }
  last_filter_us = sample_us;
  last_reading = raw;
  measurement_valid = true;
  have_valid_sample = true;
  last_valid_sample_ms = now_ms;
  sample_sequence++;
  integrateConversion(raw, now_ms);
}

void servicePersistence(uint32_t now_ms) {
  if (!state_dirty || zero_calibrating ||
      now_ms - last_persist_attempt_ms < kPersistPeriodMs) {
    return;
  }
  last_persist_attempt_ms = now_ms;
  if (!writePersistentState(gauge_state, now_ms)) {
    storage_error_count++;
  }
}

void sendStatus() {
  StatusPayload status{};
  const uint32_t now_ms = millis();
  status.uptime_ms = now_ms;
  status.flags = kStatusProtocolReady;
  if (sensor_identified) {
    status.flags |= kStatusSensorIdentified;
  }
  if (measurement_valid && sensor_ready) {
    status.flags |= kStatusMeasurementValid;
  }
  if (gauge_state.soc_known != 0U) {
    status.flags |= kStatusSocKnown;
  }
  if (storage_mounted) {
    status.flags |= kStatusStorageMounted;
  }
  if (zero_calibrating) {
    status.flags |= kStatusZeroCalibrating;
  }
  if (state_dirty) {
    status.flags |= kStatusStorageDirty;
  }
  if (command_busy) {
    status.flags |= kStatusCommandBusy;
  }
  status.sensor_state = sensor_state;
  status.ina_address = ina_address;
  status.manufacturer_id = ina_manufacturer_id;
  status.device_id = ina_device_id;
  status.capacity_mAh = gauge_state.capacity_mAh;
  status.remaining_uAs =
      gauge_state.soc_known != 0U ? gauge_state.remaining_uAs : -1;
  status.soc_centi_percent = socCentiPercent();
  status.current_offset_uA = gauge_state.current_offset_counts * kCurrentLsbUa;
  status.charge_trip_uAs = gauge_state.charge_trip_uAs;
  status.discharge_trip_uAs = gauge_state.discharge_trip_uAs;
  status.charge_trip_mWs = gauge_state.charge_trip_mWs;
  status.discharge_trip_mWs = gauge_state.discharge_trip_mWs;
  status.active_time_ms = gauge_state.active_time_ms;
  status.storage_sequence = gauge_state.sequence;
  status.i2c_error_count = i2c_error_count;
  status.storage_error_count = storage_error_count;
  status.sample_sequence = sample_sequence;
  status.last_valid_age_ms =
      have_valid_sample ? now_ms - last_valid_sample_ms : UINT32_MAX;
  status.accumulator_fault_count = accumulator_fault_count;

  if (sensor_identified) {
    status.config_register = ina_config_register;
    status.adc_config_register = ina_adc_config_register;
    status.shunt_cal_register = ina_shunt_cal_register;
    status.diag_alert = ina_diag_alert;
  }
  if (measurement_valid && sensor_ready) {
    status.vbus_raw24 = last_reading.vbus_raw24;
    status.vshunt_raw24 = last_reading.vshunt_raw24;
    status.current_raw24 = last_reading.current_raw24;
    status.bus_voltage_uV =
        roundToInt64(filtered_bus_v * 1000000.0);
    status.shunt_voltage_nV =
        roundToInt64(filtered_shunt_v * 1000000000.0);
    status.current_uA = roundToInt64(filtered_current_a * 1000000.0);
    status.power_mW = roundToInt64(
        static_cast<double>(filtered_bus_v) * status.current_uA /
        1000.0);
    status.die_temperature_mC = roundToInt64(
        static_cast<float>(last_reading.die_temperature_counts) *
        kDieTemperatureLsbMilliC);
  }
  sendFrame(kMessageStatus, &status, sizeof(status));
}

bool sendFrame(uint8_t message_type, const void* payload, size_t payload_size) {
  if (payload_size > kProtocolMaxPayload) {
    return false;
  }
  uint8_t frame[kProtocolMaxFrame]{};
  frame[0] = 0xA5U;
  frame[1] = 0x5AU;
  frame[2] = 1U;
  frame[3] = message_type;
  putU16Le(&frame[4], static_cast<uint16_t>(payload_size));
  putU32Le(&frame[6], tx_sequence++);
  memcpy(&frame[kProtocolHeaderBytes], payload, payload_size);
  putU32Le(&frame[kProtocolHeaderBytes + payload_size],
           crc32(&frame[2], 8U + payload_size));
  const size_t frame_size =
      kProtocolHeaderBytes + payload_size + kProtocolCrcBytes;
  // Arduino-Pico SerialUSB::write may wait for up to one second when a
  // connected host does not drain the CDC endpoint. Never enter that wait in
  // the measurement loop; the TUI retries commands with the same request id.
  if (static_cast<size_t>(Serial.availableForWrite()) < frame_size) {
    return false;
  }
  return Serial.write(frame, frame_size) == frame_size;
}

}  // namespace

void setup() {
  initializeDefaultState(gauge_state);

  Serial.begin(115200);
  Wire1.setSDA(kSdaPin);
  Wire1.setSCL(kSclPin);
  Wire1.setClock(kI2cClockHz);
  Wire1.begin();

  loadPersistentState();
  last_persist_attempt_ms = millis();
  last_probe_ms = millis() - kProbePeriodMs;
  last_status_ms = millis() - kStatusPeriodMs;

  connectIna228();
}

void loop() {
  processUsb();
  const uint32_t now_ms = millis();

  if (!sensor_ready) {
    if (now_ms - last_probe_ms >= kProbePeriodMs) {
      last_probe_ms = now_ms;
      connectIna228();
    }
  } else if (!zero_calibrating) {
    if (recheckIna228Configuration(now_ms)) {
      pollConversion(now_ms);
    }
  }

  servicePersistence(now_ms);
  if (now_ms - last_status_ms >= kStatusPeriodMs) {
    last_status_ms = now_ms;
    sendStatus();
  }
}
