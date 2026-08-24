#include "board_uart_protocol.h"

static void put_u16_le(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFFu);
  data[1] = (uint8_t)(value >> 8);
}

static uint16_t get_u16_le(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void put_u32_le(uint8_t *data, uint32_t value)
{
  data[0] = (uint8_t)(value & 0xFFu);
  data[1] = (uint8_t)((value >> 8) & 0xFFu);
  data[2] = (uint8_t)((value >> 16) & 0xFFu);
  data[3] = (uint8_t)(value >> 24);
}

static uint32_t get_u32_le(const uint8_t *data)
{
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8) |
         ((uint32_t)data[2] << 16) |
         ((uint32_t)data[3] << 24);
}

uint8_t BoardUart_Crc8(const uint8_t *data, uint8_t length)
{
  uint8_t crc = 0x00u;

  for (uint8_t i = 0u; i < length; ++i)
  {
    crc ^= data[i];
    for (uint8_t bit = 0u; bit < 8u; ++bit)
    {
      crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u)
                          : (uint8_t)(crc << 1);
    }
  }
  return crc;
}

size_t BoardUart_EncodeFrame(uint8_t message_type,
                             uint8_t sequence,
                             const uint8_t *payload,
                             uint8_t payload_length,
                             uint8_t *frame,
                             size_t frame_capacity)
{
  const size_t frame_size =
      (size_t)payload_length + BOARD_UART_FRAME_OVERHEAD;

  if ((payload == NULL) || (frame == NULL) ||
      (frame_capacity < frame_size))
  {
    return 0u;
  }

  frame[0] = BOARD_UART_SOF1;
  frame[1] = BOARD_UART_SOF2;
  frame[2] = message_type;
  frame[3] = payload_length;
  frame[4] = sequence;
  for (uint8_t i = 0u; i < payload_length; ++i)
  {
    frame[5u + i] = payload[i];
  }
  frame[5u + payload_length] =
      BoardUart_Crc8(&frame[2], (uint8_t)(3u + payload_length));
  return frame_size;
}

void BoardUart_EncodeDriveDetailPayload(
    const BoardUartDriveDetail_t *detail,
    uint8_t payload[BOARD_UART_DRIVE_DETAIL_PAYLOAD_LEN])
{
  if ((detail == NULL) || (payload == NULL))
  {
    return;
  }

  payload[0] = detail->flags;
  payload[1] = detail->pedal_percent;
  put_u16_le(&payload[2], detail->bus_voltage_dv);
  put_u16_le(&payload[4], (uint16_t)detail->bus_current_da);
  put_u16_le(&payload[6], (uint16_t)detail->controller_temperature_dc);
  put_u16_le(&payload[8], (uint16_t)detail->motor_temperature_dc);
  put_u32_le(&payload[10], detail->motor_fault_bits);
}

bool BoardUart_DecodeDriveDetailPayload(
    const uint8_t *payload,
    uint8_t payload_length,
    BoardUartDriveDetail_t *detail)
{
  if ((payload == NULL) || (detail == NULL) ||
      (payload_length != BOARD_UART_DRIVE_DETAIL_PAYLOAD_LEN))
  {
    return false;
  }

  detail->flags = payload[0];
  detail->pedal_percent = payload[1];
  detail->bus_voltage_dv = get_u16_le(&payload[2]);
  detail->bus_current_da = (int16_t)get_u16_le(&payload[4]);
  detail->controller_temperature_dc = (int16_t)get_u16_le(&payload[6]);
  detail->motor_temperature_dc = (int16_t)get_u16_le(&payload[8]);
  detail->motor_fault_bits = get_u32_le(&payload[10]);
  return true;
}
