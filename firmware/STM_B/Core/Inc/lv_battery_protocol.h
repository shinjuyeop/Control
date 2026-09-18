#ifndef LV_BATTERY_PROTOCOL_H
#define LV_BATTERY_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LV_BATTERY_SOF1 0xA5u
#define LV_BATTERY_SOF2 0x5Au
#define LV_BATTERY_PROTOCOL_VERSION 1u
#define LV_BATTERY_MSG_STATUS 0x01u
#define LV_BATTERY_MSG_COMMAND 0x10u
#define LV_BATTERY_MSG_COMMAND_ACK 0x11u

#define LV_BATTERY_STATUS_PAYLOAD_SIZE 136u
#define LV_BATTERY_COMMAND_PAYLOAD_SIZE 12u
#define LV_BATTERY_ACK_PAYLOAD_SIZE 16u
#define LV_BATTERY_HEADER_SIZE 10u
#define LV_BATTERY_CRC_SIZE 4u
#define LV_BATTERY_MAX_FRAME_SIZE \
  (LV_BATTERY_HEADER_SIZE + LV_BATTERY_STATUS_PAYLOAD_SIZE + \
   LV_BATTERY_CRC_SIZE)

#define LV_BATTERY_STATUS_SENSOR_IDENTIFIED (1u << 0)
#define LV_BATTERY_STATUS_MEASUREMENT_VALID (1u << 1)
#define LV_BATTERY_STATUS_SOC_KNOWN         (1u << 2)
#define LV_BATTERY_STATUS_STORAGE_MOUNTED   (1u << 3)
#define LV_BATTERY_STATUS_PROTOCOL_READY    (1u << 7)

typedef struct
{
  uint32_t frame_sequence;
  uint32_t uptime_ms;
  uint8_t flags;
  uint8_t sensor_state;
  uint8_t ina_address;
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
} LvBatteryStatus;

typedef struct
{
  uint32_t request_id;
  uint8_t command;
  int32_t value;
} LvBatteryCommand;

typedef struct
{
  uint32_t request_id;
  uint8_t command;
  uint8_t result;
  uint32_t storage_sequence;
  int32_t detail;
} LvBatteryAck;

typedef struct
{
  uint8_t data[LV_BATTERY_MAX_FRAME_SIZE];
  uint16_t index;
  uint16_t expected_size;
} LvBatteryParser;

typedef enum
{
  LV_BATTERY_PARSE_NONE = 0,
  LV_BATTERY_PARSE_STATUS,
  LV_BATTERY_PARSE_ACK,
  LV_BATTERY_PARSE_CRC_ERROR,
  LV_BATTERY_PARSE_FORMAT_ERROR
} LvBatteryParseResult;

void LvBatteryParser_Init(LvBatteryParser *parser);

LvBatteryParseResult LvBatteryParser_Feed(
    LvBatteryParser *parser,
    uint8_t byte,
    LvBatteryStatus *status,
    LvBatteryAck *ack);

bool LvBattery_DecodeCommandPayload(
    const uint8_t payload[LV_BATTERY_COMMAND_PAYLOAD_SIZE],
    LvBatteryCommand *command);

void LvBattery_EncodeStatusPayload(
    const LvBatteryStatus *status,
    uint8_t payload[LV_BATTERY_STATUS_PAYLOAD_SIZE]);

void LvBattery_EncodeAckPayload(
    const LvBatteryAck *ack,
    uint8_t payload[LV_BATTERY_ACK_PAYLOAD_SIZE]);

size_t LvBattery_EncodeCommandFrame(
    const LvBatteryCommand *command,
    uint32_t frame_sequence,
    uint8_t frame[LV_BATTERY_MAX_FRAME_SIZE]);

uint32_t LvBattery_Crc32(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* LV_BATTERY_PROTOCOL_H */
