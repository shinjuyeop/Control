#ifndef BOARD_UART_PROTOCOL_H
#define BOARD_UART_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define BOARD_UART_SOF1 0xAAu
#define BOARD_UART_SOF2 0x55u

#define BOARD_UART_MSG_A_TO_B_VEHICLE_STATUS 0x01u
#define BOARD_UART_MSG_B_TO_A_SAFETY_STATUS  0x02u
#define BOARD_UART_MSG_A_TO_B_DRIVE_DETAIL   0x03u

#define BOARD_UART_VEHICLE_PAYLOAD_LEN     3u
#define BOARD_UART_SAFETY_PAYLOAD_LEN      1u
#define BOARD_UART_DRIVE_DETAIL_PAYLOAD_LEN 14u
#define BOARD_UART_FRAME_OVERHEAD          6u
#define BOARD_UART_MAX_FRAME_SIZE \
  (BOARD_UART_DRIVE_DETAIL_PAYLOAD_LEN + BOARD_UART_FRAME_OVERHEAD)

#define BOARD_UART_DETAIL_PART1_VALID        (1u << 0)
#define BOARD_UART_DETAIL_PART2_VALID        (1u << 1)
#define BOARD_UART_DETAIL_TRACTIVE_COMMANDED (1u << 2)
#define BOARD_UART_DETAIL_TRACTIVE_FEEDBACK  (1u << 3)
#define BOARD_UART_DETAIL_COMMAND_RECEIVED   (1u << 4)
#define BOARD_UART_DETAIL_COMMAND_TIMEOUT    (1u << 5)
#define BOARD_UART_DETAIL_DRIVE_STOP         (1u << 6)

typedef struct
{
  uint8_t flags;
  uint8_t pedal_percent;
  uint16_t bus_voltage_dv;
  int16_t bus_current_da;
  int16_t controller_temperature_dc;
  int16_t motor_temperature_dc;
  uint32_t motor_fault_bits;
} BoardUartDriveDetail_t;

uint8_t BoardUart_Crc8(const uint8_t *data, uint8_t length);

size_t BoardUart_EncodeFrame(uint8_t message_type,
                             uint8_t sequence,
                             const uint8_t *payload,
                             uint8_t payload_length,
                             uint8_t *frame,
                             size_t frame_capacity);

void BoardUart_EncodeDriveDetailPayload(
    const BoardUartDriveDetail_t *detail,
    uint8_t payload[BOARD_UART_DRIVE_DETAIL_PAYLOAD_LEN]);

bool BoardUart_DecodeDriveDetailPayload(
    const uint8_t *payload,
    uint8_t payload_length,
    BoardUartDriveDetail_t *detail);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_UART_PROTOCOL_H */
