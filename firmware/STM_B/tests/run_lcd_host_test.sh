#!/bin/sh
set -eu

test_binary=$(mktemp /tmp/stm_b_lcd_host_test.XXXXXX)
uart_test_binary=$(mktemp /tmp/board_uart_protocol_host_test.XXXXXX)
trap 'rm -f "$test_binary" "$uart_test_binary"' EXIT HUP INT TERM

cc -std=c11 -Wall -Wextra -Werror \
  -Itests/mocks -ICore/Inc \
  tests/lcd_host_test.c Core/Src/lcd.c \
  -o "$test_binary"

"$test_binary"

cc -std=c11 -Wall -Wextra -Werror \
  -I../common \
  tests/board_uart_protocol_host_test.c ../common/board_uart_protocol.c \
  -o "$uart_test_binary"

"$uart_test_binary"
