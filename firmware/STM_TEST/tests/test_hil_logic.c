#include "hil_logic.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void test_pedal_conversion(void)
{
  assert(HilLogic_PedalPercentToDac(0.0f) == 560u);
  assert(HilLogic_PedalPercentToDac(100.0f) == 3000u);
  assert(HilLogic_PedalPercentToDac(-1.0f) == 560u);
  assert(HilLogic_PedalPercentToDac(101.0f) == 3000u);
}

static void test_brake_classification(void)
{
  assert(HilLogic_ClassifyBrakePulse(1693u) == HIL_BRAKE_ENGAGED);
  assert(HilLogic_ClassifyBrakePulse(1953u) == HIL_BRAKE_RELEASED);
  assert(HilLogic_ClassifyBrakePulse(1500u) == HIL_BRAKE_UNKNOWN);
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

static void encode_ez_command(uint8_t data[8], int32_t rpm, float current_a,
                              bool running, bool speed_control)
{
  const uint16_t current_raw = (uint16_t)((current_a + 3200.0f) / 0.1f);
  const uint16_t rpm_raw = (uint16_t)(rpm + 32000);
  data[0] = (uint8_t)current_raw;
  data[1] = (uint8_t)(current_raw >> 8);
  data[2] = (uint8_t)rpm_raw;
  data[3] = (uint8_t)(rpm_raw >> 8);
  data[4] = (uint8_t)((running ? 0x01u : 0x00u) |
                      (speed_control ? 0x02u : 0x00u));
  data[5] = 0u;
  data[6] = 0u;
  data[7] = 0u;
}

static void test_ez_command_decode(void)
{
  uint8_t handshake[8];
  uint8_t data[8];
  HilEZCommand_t command;

  memset(handshake, 0xAA, sizeof(handshake));
  command = HilLogic_EZDecodeCommand(handshake);
  assert(command.handshake_ack);
  assert(fabsf(HilLogic_EZPlantTargetRpm(&command)) < 0.001f);

  /* Speed control follows the commanded RPM directly. */
  encode_ez_command(data, 1500, 300.0f, true, true);
  command = HilLogic_EZDecodeCommand(data);
  assert(!command.handshake_ack && command.running && command.speed_control);
  assert(fabsf(command.rpm_field - 1500.0f) < 0.001f);
  assert(fabsf(HilLogic_EZPlantTargetRpm(&command) - 1500.0f) < 0.001f);

  /* RUN bit clear means no drive whatever the RPM field says. */
  encode_ez_command(data, 1500, 300.0f, false, true);
  command = HilLogic_EZDecodeCommand(data);
  assert(fabsf(HilLogic_EZPlantTargetRpm(&command)) < 0.001f);

  /* Torque control scales the RPM limit by the commanded phase current. */
  encode_ez_command(data, 4500, 240.0f, true, false);
  command = HilLogic_EZDecodeCommand(data);
  assert(fabsf(command.phase_current_a - 240.0f) < 0.05f);
  assert(fabsf(HilLogic_EZPlantTargetRpm(&command) - 2250.0f) < 1.0f);

  /* Zero pedal is zero current, so the RPM limit alone must not drive. */
  encode_ez_command(data, 4500, 0.0f, true, false);
  command = HilLogic_EZDecodeCommand(data);
  assert(fabsf(HilLogic_EZPlantTargetRpm(&command)) < 1.0f);

  /* Reverse uses a negative RPM limit with positive-magnitude current. */
  encode_ez_command(data, -1500, 240.0f, true, false);
  command = HilLogic_EZDecodeCommand(data);
  assert(HilLogic_EZPlantTargetRpm(&command) < -700.0f);
}

static void test_ez_plant_step(void)
{
  /* One time constant of steps reaches roughly 1 - 1/e of the target. */
  float rpm = 0.0f;
  for (int i = 0; i < 60; ++i) {
    rpm = HilLogic_PlantStepRpm(rpm, 1000.0f, 0.01f, 0.60f);
  }
  assert(rpm > 600.0f && rpm < 680.0f);

  /* It converges and never overshoots. */
  for (int i = 0; i < 1000; ++i) {
    rpm = HilLogic_PlantStepRpm(rpm, 1000.0f, 0.01f, 0.60f);
    assert(rpm <= 1000.0f);
  }
  assert(fabsf(rpm - 1000.0f) < 1.0f);

  /* Degenerate inputs hold or snap rather than diverge. */
  assert(fabsf(HilLogic_PlantStepRpm(500.0f, 0.0f, 0.0f, 0.60f) - 500.0f) < 0.001f);
  assert(fabsf(HilLogic_PlantStepRpm(500.0f, 100.0f, 0.10f, 0.01f) - 100.0f) < 0.001f);

  /* Speed and RPM stay consistent through the shared conversion. */
  HilState_t state = {0};
  /* 1000 motor rpm / 4.60 gear on a 0.47 m tire is about 19.3 kph. */
  HilLogic_SetMotionFromRpm(&state, 1000.0f);
  assert(state.vehicle_speed_kph > 19.0f && state.vehicle_speed_kph < 19.5f);
  HilLogic_SetMotionFromRpm(&state, 0.0f);
  assert(fabsf(state.vehicle_speed_kph) < 0.001f);
}

static void test_ez_square_fault(void)
{
  HilEZSquareFault_t fault = {0};
  HilEZCommand_t command = {.running = true, .speed_control = false,
                            .rpm_field = 4500.0f, .phase_current_a = 12.0f};
  HilEZFeedback_t out;

  /* Held at or above 12 A while rolling: no collapse, current follows. */
  fault.previous_drive = true;
  out = HilLogic_EZSquareFeedback(&fault, &command, 1000.0f, 0.0f);
  assert(fault.collapses == 0u && fabsf(out.phase_current_a - 12.0f) < 0.001f);
  assert(fabsf(out.rpm - 1000.0f) < 0.001f);

  /* Reapply after 0 A while rolling: spike, RPM 0, sign flip, recovery. */
  command.phase_current_a = 0.0f;
  out = HilLogic_EZSquareFeedback(&fault, &command, 500.0f, 0.0f);
  assert(fault.collapses == 0u && fabsf(out.phase_current_a) < 0.001f);
  command.phase_current_a = 2.0f;
  out = HilLogic_EZSquareFeedback(&fault, &command, 500.0f, 0.0f);
  assert(fault.collapses == 1u);
  assert(out.phase_current_a < -100.0f && out.bus_current_a > 100.0f);
  out = HilLogic_EZSquareFeedback(&fault, &command, 500.0f, 0.0f);
  assert(fabsf(out.rpm) < 0.001f);
  out = HilLogic_EZSquareFeedback(&fault, &command, 500.0f, 0.0f);
  assert(out.rpm < -100.0f);
  /* Back to the plant; 2 A sits in the dead zone and reads exactly zero. */
  command.phase_current_a = 2.0f;
  fault.previous_drive = true;
  out = HilLogic_EZSquareFeedback(&fault, &command, 300.0f, 0.0f);
  assert(fault.collapses == 1u && fabsf(out.rpm - 300.0f) < 0.001f);
  assert(fabsf(out.phase_current_a) < 0.001f);

  /* A small held current at speed collapses too; standstill never does. */
  command.phase_current_a = 8.0f;
  out = HilLogic_EZSquareFeedback(&fault, &command, 1200.0f, 0.0f);
  assert(fault.collapses == 2u);
  for (int i = 0; i < 3; ++i) (void)HilLogic_EZSquareFeedback(&fault, &command, 0.0f, 0.0f);
  out = HilLogic_EZSquareFeedback(&fault, &command, 0.0f, 0.0f);
  assert(fault.collapses == 2u && fabsf(out.phase_current_a - 8.0f) < 0.001f);

  /* Native speed control: the EZ owns its current; report the injected value. */
  command.speed_control = true;
  out = HilLogic_EZSquareFeedback(&fault, &command, 1200.0f, 33.0f);
  assert(fault.collapses == 2u && fabsf(out.phase_current_a - 33.0f) < 0.001f);

  /* Bus and phase are encoded separately for the model. */
  HilState_t state = {0};
  uint8_t part1[8];
  state.motor_voltage_v = 52.3f;
  const HilEZFeedback_t feedback = {150.0f, -120.0f, 0.0f};
  HilLogic_EZEncodePart1Feedback(&state, &feedback, part1);
  assert(part1[2] == 0xDCu && part1[3] == 0x82u);   /* (150+3200)*10 = 33500 */
  assert(part1[4] == 0x50u && part1[5] == 0x78u);   /* (-120+3200)*10 = 30800 */
  assert(part1[6] == 0x00u && part1[7] == 0x7Du);   /* 0 rpm = 32000 */

  char line[64], reply[128];
  (void)snprintf(line, sizeof(line), "SET EZ_SQUARE ON");
  assert(HilLogic_ProcessCommand(&state, line, reply, sizeof(reply)));
  assert(state.ez_square_fault);
  (void)snprintf(line, sizeof(line), "set ez_square off");
  assert(HilLogic_ProcessCommand(&state, line, reply, sizeof(reply)));
  assert(!state.ez_square_fault);
  (void)snprintf(line, sizeof(line), "SET EZ_SQUARE MAYBE");
  assert(!HilLogic_ProcessCommand(&state, line, reply, sizeof(reply)));
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
  char reply[768];
  char command1[] = "SET MODE AUTO";
  char command2[] = "SET PEDAL 30";
  char command3[] = "SET PEDAL 101";
  char command4[] = "SET EZ_CAN OFF";
  char command5[] = "SET SPEED 10.5";
  char plant_on[] = "SET PLANT ON";
  char plant_off[] = "SET PLANT OFF";
  char plant_bad[] = "SET PLANT MAYBE";
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
  assert(HilLogic_ProcessCommand(&state, plant_on, reply, sizeof(reply)));
  assert(state.ez_plant_enabled);
  /* Disabling keeps the motion the loop had reached. */
  assert(HilLogic_ProcessCommand(&state, plant_off, reply, sizeof(reply)));
  assert(!state.ez_plant_enabled);
  assert(state.motor_rpm > 0.0f);
  assert(!HilLogic_ProcessCommand(&state, plant_bad, reply, sizeof(reply)));
  assert(strncmp(reply, "ERR ", 4u) == 0);
  state.ez_rx_count = UINT32_MAX;
  state.ez_tx_count = UINT32_MAX;
  state.keya_rx_count = UINT32_MAX;
  state.keya_tx_count = UINT32_MAX;
  state.ez_rx_age_ms = UINT32_MAX;
  state.keya_rx_age_ms = UINT32_MAX;
  state.pedal_percent = 100.0f;
  state.vehicle_speed_kph = -200.0f;
  state.motor_rpm = -32000.0f;
  state.motor_voltage_v = 300.0f;
  state.motor_current_a = -3200.0f;
  state.motor_temperature_c = -40.0f;
  state.steering_position = -1800.0f;
  state.steering_target = -1800.0f;
  state.ez_target_rpm = -32000.0f;
  state.ez_command_rpm = -32000.0f;
  state.ez_command_current_a = -3200.0f;
  /* Widest FDCAN protocol-status values, so the length assert below covers the
   * worst-case STATUS line against HIL_UART_REPLY_SIZE. */
  state.ez_bus_off = true;
  state.ez_error_passive = true;
  state.ez_warning = true;
  state.ez_tec = 255u;
  state.ez_rec = 255u;
  state.keya_bus_off = true;
  state.keya_error_passive = true;
  state.keya_warning = true;
  state.keya_tec = 255u;
  state.keya_rec = 255u;
  assert(HilLogic_ProcessCommand(&state, status, reply, sizeof(reply)));
  assert(strstr(reply, "STATUS protocol=3 mode=AUTO") != NULL);
  assert(strstr(reply, "protocol=3") != NULL);
  assert(strstr(reply, "plant=0") != NULL);
  assert(strstr(reply, "ez_target_rpm=") != NULL);
  assert(strstr(reply, "ez_command_rpm=") != NULL);
  assert(strstr(reply, "keya_rx_age_ms=") != NULL);
  assert(strstr(reply, "ez_busoff=1") != NULL);
  assert(strstr(reply, "ez_tec=255") != NULL);
  assert(strstr(reply, "keya_busoff=1") != NULL);
  assert(strstr(reply, "keya_errpass=1") != NULL);
  assert(strstr(reply, "keya_rec=255") != NULL);
  assert(strlen(reply) < sizeof(reply) - 1u);
}

int main(void)
{
  test_pedal_conversion();
  test_brake_classification();
  test_steering_plant();
  test_ezkontrol_encoding();
  test_ez_command_decode();
  test_ez_plant_step();
  test_ez_square_fault();
  test_keya_decode();
  test_command_parser();
  puts("hil_logic tests: PASS");
  return 0;
}
