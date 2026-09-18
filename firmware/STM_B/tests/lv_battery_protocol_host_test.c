#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "lv_battery_protocol.h"

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

int main(void)
{
  uint8_t frame[LV_BATTERY_MAX_FRAME_SIZE] = {0};
  uint8_t *payload = &frame[LV_BATTERY_HEADER_SIZE];
  frame[0] = LV_BATTERY_SOF1;
  frame[1] = LV_BATTERY_SOF2;
  frame[2] = LV_BATTERY_PROTOCOL_VERSION;
  frame[3] = LV_BATTERY_MSG_STATUS;
  put_u16_le(&frame[4], LV_BATTERY_STATUS_PAYLOAD_SIZE);
  put_u32_le(&frame[6], 42u);
  put_u32_le(&payload[0], 1234u);
  payload[4] = LV_BATTERY_STATUS_MEASUREMENT_VALID |
               LV_BATTERY_STATUS_SOC_KNOWN;
  put_u32_le(&payload[32], 13200000u);
  put_u32_le(&payload[40], (uint32_t)(int32_t)-2500000);
  put_u32_le(&payload[56], 7500u);
  put_u64_le(&payload[60], 270000000000ULL);
  put_u32_le(&payload[68], 100000u);
  put_u32_le(&frame[LV_BATTERY_HEADER_SIZE +
                    LV_BATTERY_STATUS_PAYLOAD_SIZE],
             LvBattery_Crc32(&frame[2],
                 8u + LV_BATTERY_STATUS_PAYLOAD_SIZE));

  LvBatteryParser parser;
  LvBatteryStatus status;
  LvBatteryAck ack;
  LvBatteryParser_Init(&parser);
  LvBatteryParseResult result = LV_BATTERY_PARSE_NONE;
  for (size_t i = 0u; i < sizeof(frame); ++i) {
    result = LvBatteryParser_Feed(&parser, frame[i], &status, &ack);
  }
  assert(result == LV_BATTERY_PARSE_STATUS);
  assert(status.frame_sequence == 42u);
  assert(status.uptime_ms == 1234u);
  assert(status.bus_voltage_uV == 13200000);
  assert(status.current_uA == -2500000);
  assert(status.soc_centi_percent == 7500);
  assert(status.remaining_uAs == 270000000000LL);
  assert(status.capacity_mAh == 100000u);
  uint8_t encoded_status[LV_BATTERY_STATUS_PAYLOAD_SIZE];
  LvBattery_EncodeStatusPayload(&status, encoded_status);
  assert(memcmp(encoded_status, payload, sizeof(encoded_status)) == 0);

  frame[20] ^= 0x01u;
  LvBatteryParser_Init(&parser);
  for (size_t i = 0u; i < sizeof(frame); ++i) {
    result = LvBatteryParser_Feed(&parser, frame[i], &status, &ack);
  }
  assert(result == LV_BATTERY_PARSE_CRC_ERROR);

  const uint8_t command_payload[LV_BATTERY_COMMAND_PAYLOAD_SIZE] = {
      0x78, 0x56, 0x34, 0x12, 4, 0, 0, 0, 0xA0, 0x86, 0x01, 0};
  LvBatteryCommand command;
  assert(LvBattery_DecodeCommandPayload(command_payload, &command));
  assert(command.request_id == 0x12345678u);
  assert(command.command == 4u && command.value == 100000);
  const size_t command_size =
      LvBattery_EncodeCommandFrame(&command, 7u, frame);
  assert(command_size == 26u);
  assert(frame[3] == LV_BATTERY_MSG_COMMAND);
  assert(memcmp(&frame[LV_BATTERY_HEADER_SIZE], command_payload,
                sizeof(command_payload)) == 0);

  memset(frame, 0, sizeof(frame));
  frame[0] = LV_BATTERY_SOF1;
  frame[1] = LV_BATTERY_SOF2;
  frame[2] = LV_BATTERY_PROTOCOL_VERSION;
  frame[3] = LV_BATTERY_MSG_COMMAND_ACK;
  put_u16_le(&frame[4], LV_BATTERY_ACK_PAYLOAD_SIZE);
  put_u32_le(&frame[6], 8u);
  put_u32_le(&frame[10], 0x12345678u);
  frame[14] = 4u;
  frame[15] = 0u;
  put_u32_le(&frame[18], 55u);
  put_u32_le(&frame[22], 100000u);
  put_u32_le(&frame[26], LvBattery_Crc32(&frame[2], 24u));
  LvBatteryParser_Init(&parser);
  result = LV_BATTERY_PARSE_NONE;
  for (size_t i = 0u; i < 30u; ++i)
    result = LvBatteryParser_Feed(&parser, frame[i], &status, &ack);
  assert(result == LV_BATTERY_PARSE_ACK);
  assert(ack.request_id == 0x12345678u && ack.command == 4u);
  assert(ack.result == 0u && ack.storage_sequence == 55u);
  assert(ack.detail == 100000);
  return 0;
}
