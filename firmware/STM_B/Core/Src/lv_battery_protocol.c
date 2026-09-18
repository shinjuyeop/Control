#include "lv_battery_protocol.h"

#include <string.h>

static uint16_t get_u16_le(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static uint32_t get_u32_le(const uint8_t *data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) |
         ((uint32_t)data[3] << 24);
}

static uint64_t get_u64_le(const uint8_t *data)
{
  return (uint64_t)get_u32_le(data) |
         ((uint64_t)get_u32_le(data + 4) << 32);
}

static void put_u16_le(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8);
}

static void put_u32_le(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8);
  data[2] = (uint8_t)(value >> 16);
  data[3] = (uint8_t)(value >> 24);
}

static void put_u64_le(uint8_t *data, uint64_t value)
{
  put_u32_le(data, (uint32_t)value);
  put_u32_le(data + 4, (uint32_t)(value >> 32));
}

static void reset_parser(LvBatteryParser *parser, uint8_t last_byte)
{
  parser->index = 0u;
  parser->expected_size = 0u;
  if (last_byte == LV_BATTERY_SOF1)
  {
    parser->data[0] = last_byte;
    parser->index = 1u;
  }
}

static void decode_status(const uint8_t *payload,
                          uint32_t frame_sequence,
                          LvBatteryStatus *status)
{
  memset(status, 0, sizeof(*status));
  status->frame_sequence = frame_sequence;
  status->uptime_ms = get_u32_le(&payload[0]);
  status->flags = payload[4];
  status->sensor_state = payload[5];
  status->ina_address = payload[6];
  status->manufacturer_id = get_u16_le(&payload[8]);
  status->device_id = get_u16_le(&payload[10]);
  status->config_register = get_u16_le(&payload[12]);
  status->adc_config_register = get_u16_le(&payload[14]);
  status->shunt_cal_register = get_u16_le(&payload[16]);
  status->diag_alert = get_u16_le(&payload[18]);
  status->vbus_raw24 = get_u32_le(&payload[20]);
  status->vshunt_raw24 = get_u32_le(&payload[24]);
  status->current_raw24 = get_u32_le(&payload[28]);
  status->bus_voltage_uV = (int32_t)get_u32_le(&payload[32]);
  status->shunt_voltage_nV = (int32_t)get_u32_le(&payload[36]);
  status->current_uA = (int32_t)get_u32_le(&payload[40]);
  status->power_mW = (int32_t)get_u32_le(&payload[44]);
  status->current_offset_uA = (int32_t)get_u32_le(&payload[48]);
  status->die_temperature_mC = (int32_t)get_u32_le(&payload[52]);
  status->soc_centi_percent = (int32_t)get_u32_le(&payload[56]);
  status->remaining_uAs = (int64_t)get_u64_le(&payload[60]);
  status->capacity_mAh = get_u32_le(&payload[68]);
  status->charge_trip_uAs = get_u64_le(&payload[72]);
  status->discharge_trip_uAs = get_u64_le(&payload[80]);
  status->charge_trip_mWs = get_u64_le(&payload[88]);
  status->discharge_trip_mWs = get_u64_le(&payload[96]);
  status->active_time_ms = get_u64_le(&payload[104]);
  status->storage_sequence = get_u32_le(&payload[112]);
  status->i2c_error_count = get_u32_le(&payload[116]);
  status->storage_error_count = get_u32_le(&payload[120]);
  status->sample_sequence = get_u32_le(&payload[124]);
  status->last_valid_age_ms = get_u32_le(&payload[128]);
  status->accumulator_fault_count = get_u32_le(&payload[132]);
}

static void decode_ack(const uint8_t *payload, LvBatteryAck *ack)
{
  ack->request_id = get_u32_le(&payload[0]);
  ack->command = payload[4];
  ack->result = payload[5];
  ack->storage_sequence = get_u32_le(&payload[8]);
  ack->detail = (int32_t)get_u32_le(&payload[12]);
}

uint32_t LvBattery_Crc32(const uint8_t *data, size_t length)
{
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t index = 0u; index < length; ++index)
  {
    crc ^= data[index];
    for (uint8_t bit = 0u; bit < 8u; ++bit)
    {
      crc = ((crc & 1u) != 0u) ?
          ((crc >> 1) ^ 0xEDB88320u) : (crc >> 1);
    }
  }
  return ~crc;
}

void LvBatteryParser_Init(LvBatteryParser *parser)
{
  if (parser != NULL)
  {
    memset(parser, 0, sizeof(*parser));
  }
}

LvBatteryParseResult LvBatteryParser_Feed(
    LvBatteryParser *parser,
    uint8_t byte,
    LvBatteryStatus *status,
    LvBatteryAck *ack)
{
  if ((parser == NULL) || (status == NULL) || (ack == NULL))
  {
    return LV_BATTERY_PARSE_FORMAT_ERROR;
  }

  if (parser->index == 0u)
  {
    if (byte == LV_BATTERY_SOF1)
    {
      parser->data[0] = byte;
      parser->index = 1u;
    }
    return LV_BATTERY_PARSE_NONE;
  }

  if (parser->index == 1u)
  {
    if (byte == LV_BATTERY_SOF2)
    {
      parser->data[1] = byte;
      parser->index = 2u;
    }
    else
    {
      reset_parser(parser, byte);
    }
    return LV_BATTERY_PARSE_NONE;
  }

  if (parser->index >= sizeof(parser->data))
  {
    reset_parser(parser, byte);
    return LV_BATTERY_PARSE_FORMAT_ERROR;
  }
  parser->data[parser->index++] = byte;

  if (parser->index == 6u)
  {
    const uint16_t payload_size = get_u16_le(&parser->data[4]);
    if (payload_size > LV_BATTERY_STATUS_PAYLOAD_SIZE)
    {
      reset_parser(parser, byte);
      return LV_BATTERY_PARSE_FORMAT_ERROR;
    }
    parser->expected_size = (uint16_t)(LV_BATTERY_HEADER_SIZE +
        payload_size + LV_BATTERY_CRC_SIZE);
  }

  if ((parser->expected_size == 0u) ||
      (parser->index < parser->expected_size))
  {
    return LV_BATTERY_PARSE_NONE;
  }

  const uint16_t payload_size = get_u16_le(&parser->data[4]);
  const uint32_t received_crc = get_u32_le(
      &parser->data[LV_BATTERY_HEADER_SIZE + payload_size]);
  const uint32_t expected_crc = LvBattery_Crc32(
      &parser->data[2], 8u + payload_size);
  LvBatteryParseResult result = LV_BATTERY_PARSE_NONE;

  if (received_crc != expected_crc)
  {
    result = LV_BATTERY_PARSE_CRC_ERROR;
  }
  else if (parser->data[2] != LV_BATTERY_PROTOCOL_VERSION)
  {
    result = LV_BATTERY_PARSE_FORMAT_ERROR;
  }
  else if ((parser->data[3] == LV_BATTERY_MSG_STATUS) &&
           (payload_size == LV_BATTERY_STATUS_PAYLOAD_SIZE))
  {
    decode_status(&parser->data[LV_BATTERY_HEADER_SIZE],
                  get_u32_le(&parser->data[6]), status);
    result = LV_BATTERY_PARSE_STATUS;
  }
  else if ((parser->data[3] == LV_BATTERY_MSG_COMMAND_ACK) &&
           (payload_size == LV_BATTERY_ACK_PAYLOAD_SIZE))
  {
    decode_ack(&parser->data[LV_BATTERY_HEADER_SIZE], ack);
    result = LV_BATTERY_PARSE_ACK;
  }

  reset_parser(parser, byte);
  return result;
}

bool LvBattery_DecodeCommandPayload(
    const uint8_t payload[LV_BATTERY_COMMAND_PAYLOAD_SIZE],
    LvBatteryCommand *command)
{
  if ((payload == NULL) || (command == NULL)) return false;
  command->request_id = get_u32_le(&payload[0]);
  command->command = payload[4];
  command->value = (int32_t)get_u32_le(&payload[8]);
  return command->request_id != 0u &&
      command->command >= 1u && command->command <= 4u;
}

void LvBattery_EncodeStatusPayload(
    const LvBatteryStatus *status,
    uint8_t payload[LV_BATTERY_STATUS_PAYLOAD_SIZE])
{
  memset(payload, 0, LV_BATTERY_STATUS_PAYLOAD_SIZE);
  put_u32_le(&payload[0], status->uptime_ms);
  payload[4] = status->flags;
  payload[5] = status->sensor_state;
  payload[6] = status->ina_address;
  put_u16_le(&payload[8], status->manufacturer_id);
  put_u16_le(&payload[10], status->device_id);
  put_u16_le(&payload[12], status->config_register);
  put_u16_le(&payload[14], status->adc_config_register);
  put_u16_le(&payload[16], status->shunt_cal_register);
  put_u16_le(&payload[18], status->diag_alert);
  put_u32_le(&payload[20], status->vbus_raw24);
  put_u32_le(&payload[24], status->vshunt_raw24);
  put_u32_le(&payload[28], status->current_raw24);
  put_u32_le(&payload[32], (uint32_t)status->bus_voltage_uV);
  put_u32_le(&payload[36], (uint32_t)status->shunt_voltage_nV);
  put_u32_le(&payload[40], (uint32_t)status->current_uA);
  put_u32_le(&payload[44], (uint32_t)status->power_mW);
  put_u32_le(&payload[48], (uint32_t)status->current_offset_uA);
  put_u32_le(&payload[52], (uint32_t)status->die_temperature_mC);
  put_u32_le(&payload[56], (uint32_t)status->soc_centi_percent);
  put_u64_le(&payload[60], (uint64_t)status->remaining_uAs);
  put_u32_le(&payload[68], status->capacity_mAh);
  put_u64_le(&payload[72], status->charge_trip_uAs);
  put_u64_le(&payload[80], status->discharge_trip_uAs);
  put_u64_le(&payload[88], status->charge_trip_mWs);
  put_u64_le(&payload[96], status->discharge_trip_mWs);
  put_u64_le(&payload[104], status->active_time_ms);
  put_u32_le(&payload[112], status->storage_sequence);
  put_u32_le(&payload[116], status->i2c_error_count);
  put_u32_le(&payload[120], status->storage_error_count);
  put_u32_le(&payload[124], status->sample_sequence);
  put_u32_le(&payload[128], status->last_valid_age_ms);
  put_u32_le(&payload[132], status->accumulator_fault_count);
}

void LvBattery_EncodeAckPayload(
    const LvBatteryAck *ack,
    uint8_t payload[LV_BATTERY_ACK_PAYLOAD_SIZE])
{
  memset(payload, 0, LV_BATTERY_ACK_PAYLOAD_SIZE);
  put_u32_le(&payload[0], ack->request_id);
  payload[4] = ack->command;
  payload[5] = ack->result;
  put_u32_le(&payload[8], ack->storage_sequence);
  put_u32_le(&payload[12], (uint32_t)ack->detail);
}

size_t LvBattery_EncodeCommandFrame(
    const LvBatteryCommand *command,
    uint32_t frame_sequence,
    uint8_t frame[LV_BATTERY_MAX_FRAME_SIZE])
{
  if ((command == NULL) || (frame == NULL)) return 0u;
  uint8_t payload[LV_BATTERY_COMMAND_PAYLOAD_SIZE] = {0};
  put_u32_le(&payload[0], command->request_id);
  payload[4] = command->command;
  put_u32_le(&payload[8], (uint32_t)command->value);
  frame[0] = LV_BATTERY_SOF1;
  frame[1] = LV_BATTERY_SOF2;
  frame[2] = LV_BATTERY_PROTOCOL_VERSION;
  frame[3] = LV_BATTERY_MSG_COMMAND;
  put_u16_le(&frame[4], LV_BATTERY_COMMAND_PAYLOAD_SIZE);
  put_u32_le(&frame[6], frame_sequence);
  memcpy(&frame[LV_BATTERY_HEADER_SIZE], payload, sizeof(payload));
  put_u32_le(&frame[LV_BATTERY_HEADER_SIZE + sizeof(payload)],
      LvBattery_Crc32(&frame[2], 8u + sizeof(payload)));
  return LV_BATTERY_HEADER_SIZE + sizeof(payload) + LV_BATTERY_CRC_SIZE;
}
