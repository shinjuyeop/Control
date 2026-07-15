#!/bin/sh
set -eu

cc -std=c11 -Wall -Wextra -Werror \
  -Itests/mocks -ICore/Inc \
  tests/lcd_host_test.c Core/Src/lcd.c \
  -o /tmp/stm_b_lcd_host_test

/tmp/stm_b_lcd_host_test
