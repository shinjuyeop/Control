#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "board_uart_protocol.h"

int main(void)
{
  static const uint8_t crc_check[] = "123456789";
  const BoardUartDriveDetail_t input = {
      .flags = BOARD_UART_DETAIL_PART1_VALID |
               BOARD_UART_DETAIL_PART2_VALID |
               BOARD_UART_DETAIL_TRACTIVE_FEEDBACK,
      .pedal_percent = 37U,
      .bus_voltage_dv = 724U,
      .bus_current_da = -183,
      .phase_current_da = 2567,
      .controller_temperature_dc = 412,
      .motor_temperature_dc = 487,
      .motor_fault_bits = 0x00123456UL};
  uint8_t payload[BOARD_UART_DRIVE_DETAIL_PAYLOAD_LEN];
  uint8_t frame[BOARD_UART_MAX_FRAME_SIZE];
  BoardUartDriveDetail_t output;

  BoardUart_EncodeDriveDetailPayload(&input, payload);
  const size_t size = BoardUart_EncodeFrame(
      BOARD_UART_MSG_A_TO_B_DRIVE_DETAIL,
      0x2AU,
      payload,
      sizeof(payload),
      frame,
      sizeof(frame));

  assert(size == BOARD_UART_MAX_FRAME_SIZE);
  assert(frame[0] == BOARD_UART_SOF1);
  assert(frame[1] == BOARD_UART_SOF2);
  assert(frame[2] == BOARD_UART_MSG_A_TO_B_DRIVE_DETAIL);
  assert(frame[3] == BOARD_UART_DRIVE_DETAIL_PAYLOAD_LEN);
  assert(frame[4] == 0x2AU);
  assert(frame[7] == 0xD4U && frame[8] == 0x02U);
  assert(frame[9] == 0x49U && frame[10] == 0xFFU);
  assert(frame[11] == 0x07U && frame[12] == 0x0AU);
  assert(BoardUart_Crc8(crc_check, 9U) == 0xF4U);
  assert(frame[size - 1U] == 0x6CU);

  assert(BoardUart_DecodeDriveDetailPayload(
      &frame[5], frame[3], &output));
  assert(output.flags == input.flags);
  assert(output.pedal_percent == input.pedal_percent);
  assert(output.bus_voltage_dv == input.bus_voltage_dv);
  assert(output.bus_current_da == input.bus_current_da);
  assert(output.phase_current_da == input.phase_current_da);
  assert(output.controller_temperature_dc == input.controller_temperature_dc);
  assert(output.motor_temperature_dc == input.motor_temperature_dc);
  assert(output.motor_fault_bits == input.motor_fault_bits);
  puts("board_uart_protocol_host_test: PASS");
  return 0;
}
