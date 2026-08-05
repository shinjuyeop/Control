#include "hil_logic.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void test_pedal_conversion(void)
{
  assert(HilLogic_PedalPercentToDac(0.0f) == 560u);
  assert(HilLogic_PedalPercentToDac(100.0f) == 3100u);
  assert(HilLogic_PedalPercentToDac(-1.0f) == 560u);
  assert(HilLogic_PedalPercentToDac(101.0f) == 3100u);
}

static void test_brake_classification(void)
{
  assert(HilLogic_ClassifyBrakePulse(1194u) == HIL_BRAKE_ENGAGED);
  assert(HilLogic_ClassifyBrakePulse(1388u) == HIL_BRAKE_RELEASED);
  assert(HilLogic_ClassifyBrakePulse(1290u) == HIL_BRAKE_UNKNOWN);
}

static void test_steering_plant(void)
{
  assert(fabsf(HilLogic_Approach(0.0f, 10.0f, 0.9f) - 0.9f) < 0.001f);
  assert(fabsf(HilLogic_Approach(9.5f, 10.0f, 0.9f) - 10.0f) < 0.001f);
  assert(HilLogic_KeyaDegreesToCount(360.0f) == 10000);
  assert(fabsf(HilLogic_KeyaCountToDegrees(-5000) + 180.0f) < 0.001f);
}

static void test_ezkontrol_encoding(void)
{
  HilState_t state = {0};
  uint8_t part1[8], part2[8];
  state.motor_voltage_v = 52.3f;
  state.motor_current_a = 12.5f;
  state.motor_rpm = 500.0f;
  state.motor_temperature_c = 40.0f;
  state.ez_fault = true;
  HilLogic_EZEncodePart1(&state, part1);
  assert(part1[0] == 0x0Bu && part1[1] == 0x02u);
  assert(part1[2] == 0x7Du && part1[3] == 0x7Du);
  assert(part1[6] == 0xF4u && part1[7] == 0x7Eu);
  HilLogic_EZEncodePart2(&state, 7u, part2);
  assert(part2[0] == 80u && part2[1] == 80u);
  assert(part2[5] == 0x20u && part2[7] == 7u);
}

static void test_keya_decode(void)
{
  const uint8_t payload[8] = {0x23, 0x02, 0x20, 0x01,
                              0xFF, 0xFF, 0xFB, 0x3F};
  HilKeyaCommand_t command = HilLogic_KeyaDecode(payload);
  assert(command.type == HIL_KEYA_COMMAND_POSITION);
  assert(command.position_count == -1217);
}

static void test_command_parser(void)
{
  HilState_t state = {0};
  char reply[512];
  char command1[] = "SET MODE AUTO";
  char command2[] = "SET PEDAL 30";
  char command3[] = "SET PEDAL 101";
  char command4[] = "SET EZ_CAN OFF";
  char command5[] = "SET SPEED 10.5";
  char status[] = "STATUS";
  state.ez_can_enabled = true;
  assert(HilLogic_ProcessCommand(&state, command1, reply, sizeof(reply)));
  assert(state.auto_mode);
  assert(HilLogic_ProcessCommand(&state, command2, reply, sizeof(reply)));
  assert(fabsf(state.pedal_percent - 30.0f) < 0.001f);
  assert(!HilLogic_ProcessCommand(&state, command3, reply, sizeof(reply)));
  assert(strncmp(reply, "ERR ", 4u) == 0);
  assert(HilLogic_ProcessCommand(&state, command4, reply, sizeof(reply)));
  assert(!state.ez_can_enabled);
  assert(HilLogic_ProcessCommand(&state, command5, reply, sizeof(reply)));
  assert(state.motor_rpm > 0.0f);
  assert(HilLogic_ProcessCommand(&state, status, reply, sizeof(reply)));
  assert(strstr(reply, "STATUS mode=AUTO") != NULL);
}

int main(void)
{
  test_pedal_conversion();
  test_brake_classification();
  test_steering_plant();
  test_ezkontrol_encoding();
  test_keya_decode();
  test_command_parser();
  puts("hil_logic tests: PASS");
  return 0;
}
