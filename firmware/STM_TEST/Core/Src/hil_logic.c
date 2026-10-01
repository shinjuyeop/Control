#include "hil_logic.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define HIL_COMMAND_TOKEN_MAX 6u
#define HIL_SPEED_MIN_KPH (-200.0f)
#define HIL_SPEED_MAX_KPH 200.0f
#define HIL_RPM_MIN (-32000.0f)
#define HIL_RPM_MAX 32000.0f
#define HIL_VOLTAGE_MIN 0.0f
#define HIL_VOLTAGE_MAX 300.0f
#define HIL_CURRENT_MIN (-3200.0f)
#define HIL_CURRENT_MAX 3200.0f
#define HIL_MOTOR_TEMP_MIN (-40.0f)
#define HIL_MOTOR_TEMP_MAX 210.0f
#define HIL_STEERING_MIN_DEG (-1800.0f)
#define HIL_STEERING_MAX_DEG 1800.0f
#define HIL_EZ_TIRE_DIAMETER_M 0.47f
#define HIL_EZ_GEAR_RATIO 4.60f
#define HIL_PI_F 3.14159f

static uint16_t encode_offset_01(float value)
{
  float raw = (value + 3200.0f) * 10.0f;
  if (raw < 0.0f) raw = 0.0f;
  if (raw > 65535.0f) raw = 65535.0f;
  return (uint16_t)lroundf(raw);
}

static uint16_t encode_u01(float value)
{
  float raw = value * 10.0f;
  if (raw < 0.0f) raw = 0.0f;
  if (raw > 65535.0f) raw = 65535.0f;
  return (uint16_t)lroundf(raw);
}

static bool token_is(const char *token, const char *expected)
{
  return (token != NULL) && (strcmp(token, expected) == 0);
}

static bool parse_float(const char *token, float minimum, float maximum,
                        float *value)
{
  char *end = NULL;
  errno = 0;
  if ((token == NULL) || (value == NULL)) return false;
  float parsed = strtof(token, &end);
  if ((errno == ERANGE) || (end == token) || (*end != '\0') ||
      !isfinite(parsed) || (parsed < minimum) || (parsed > maximum)) {
    return false;
  }
  *value = parsed;
  return true;
}

static void set_error(char *reply, size_t size, const char *reason)
{
  if ((reply != NULL) && (size > 0u)) {
    (void)snprintf(reply, size, "ERR %s", reason);
  }
}

uint16_t HilLogic_PedalPercentToDac(float percent)
{
  if (!isfinite(percent) || (percent < 0.0f)) percent = 0.0f;
  if (percent > 100.0f) percent = 100.0f;
  const float raw = (float)HIL_PEDAL_ADC_MIN +
      percent * (float)(HIL_PEDAL_ADC_MAX - HIL_PEDAL_ADC_MIN) / 100.0f;
  uint32_t count = (uint32_t)lroundf(raw);
  if (count > HIL_DAC_MAX_COUNT) count = HIL_DAC_MAX_COUNT;
  return (uint16_t)count;
}

HilBrakeState_t HilLogic_ClassifyBrakePulse(uint32_t pulse_us)
{
  const uint32_t engaged_delta = (pulse_us > HIL_BRAKE_ENGAGED_PULSE_US) ?
      pulse_us - HIL_BRAKE_ENGAGED_PULSE_US :
      HIL_BRAKE_ENGAGED_PULSE_US - pulse_us;
  const uint32_t released_delta = (pulse_us > HIL_BRAKE_RELEASED_PULSE_US) ?
      pulse_us - HIL_BRAKE_RELEASED_PULSE_US :
      HIL_BRAKE_RELEASED_PULSE_US - pulse_us;
  if (engaged_delta <= HIL_BRAKE_PULSE_TOLERANCE_US) return HIL_BRAKE_ENGAGED;
  if (released_delta <= HIL_BRAKE_PULSE_TOLERANCE_US) return HIL_BRAKE_RELEASED;
  return HIL_BRAKE_UNKNOWN;
}

float HilLogic_Approach(float current, float target, float max_step)
{
  if (max_step < 0.0f) max_step = -max_step;
  if (current < target - max_step) return current + max_step;
  if (current > target + max_step) return current - max_step;
  return target;
}

void HilLogic_SetMotionFromRpm(HilState_t *state, float rpm)
{
  if (state == NULL) return;
  if (rpm < HIL_RPM_MIN) rpm = HIL_RPM_MIN;
  if (rpm > HIL_RPM_MAX) rpm = HIL_RPM_MAX;
  const float speed_mps = rpm * HIL_PI_F * HIL_EZ_TIRE_DIAMETER_M /
      (60.0f * HIL_EZ_GEAR_RATIO);
  state->motor_rpm = rpm;
  state->vehicle_speed_kph = speed_mps * 3.6f;
}

HilEZCommand_t HilLogic_EZDecodeCommand(const uint8_t data[8])
{
  HilEZCommand_t command = {0};
  if (data == NULL) return command;
  if (HilLogic_IsFilled(data, 0xAAu)) {
    command.handshake_ack = true;
    return command;
  }
  const uint16_t current_raw =
      (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
  const uint16_t rpm_raw =
      (uint16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
  command.phase_current_a = ((float)current_raw * 0.1f) - 3200.0f;
  command.rpm_field = (float)((int32_t)rpm_raw - 32000);
  command.running = (data[4] & 0x01u) != 0u;
  command.speed_control = (data[4] & 0x02u) != 0u;
  return command;
}

float HilLogic_EZPlantTargetRpm(const HilEZCommand_t *command)
{
  if ((command == NULL) || command->handshake_ack || !command->running) {
    return 0.0f;
  }
  if (command->speed_control) return command->rpm_field;

  /* Torque control: the RPM field is only a limit, so the commanded phase
   * current decides how much of it the motor actually reaches. */
  float fraction = command->phase_current_a / HIL_EZ_PLANT_TORQUE_FULL_SCALE_A;
  if (fraction < -1.0f) fraction = -1.0f;
  if (fraction > 1.0f) fraction = 1.0f;
  if (fraction < 0.0f) fraction = -fraction;
  return command->rpm_field * fraction;
}

float HilLogic_PlantStepRpm(float current_rpm, float target_rpm,
                            float dt_s, float tau_s)
{
  if (!isfinite(current_rpm) || !isfinite(target_rpm)) return current_rpm;
  if (!(dt_s > 0.0f)) return current_rpm;
  /* tau <= dt degenerates to snapping straight to the target. */
  if (!(tau_s > dt_s)) return target_rpm;
  return current_rpm + ((target_rpm - current_rpm) * (dt_s / tau_s));
}

bool HilLogic_ProcessCommand(HilState_t *state, char *line,
                             char *reply, size_t reply_size)
{
  char original[160];
  char *tokens[HIL_COMMAND_TOKEN_MAX] = {0};
  char *save = NULL;
  size_t count = 0u;
  float value = 0.0f;
  bool success = false;

  if ((state == NULL) || (line == NULL) || (reply == NULL) ||
      (reply_size == 0u)) return false;
  while (isspace((unsigned char)*line)) ++line;
  if (*line == '\0') {
    set_error(reply, reply_size, "empty_command");
    return false;
  }
  (void)snprintf(original, sizeof(original), "%s", line);
  for (char *p = line; *p != '\0'; ++p) *p = (char)toupper((unsigned char)*p);
  for (char *token = strtok_r(line, " \t", &save); token != NULL;
       token = strtok_r(NULL, " \t", &save)) {
    if (count >= HIL_COMMAND_TOKEN_MAX) {
      set_error(reply, reply_size, "too_many_arguments");
      return false;
    }
    tokens[count++] = token;
  }

  if ((count == 1u) && token_is(tokens[0], "STATUS")) {
    (void)snprintf(reply, reply_size,
      "STATUS protocol=3 mode=%s gear=%s pedal=%.1f estop_dash=%u estop_side=%u estop_rf=%u "
      "speed=%.2f rpm=%.0f voltage=%.1f current=%.1f motor_temp=%.1f "
      "brake_period_us=%lu brake_us=%lu brake=%s pwm_lost=%u "
      "steer=%.2f steer_target=%.2f ez_rx=%lu ez_tx=%lu keya_rx=%lu keya_tx=%lu "
      "ez_can=%u keya_can=%u ez_ok=%u keya_ok=%u ez_fault=%u keya_fault=%u "
      "plant=%u ez_target_rpm=%.0f ez_running=%u ez_speed_control=%u "
      "ez_command_rpm=%.0f ez_command_current=%.1f ez_rx_age_ms=%lu "
      "keya_enabled=%u keya_rx_age_ms=%lu "
      "ez_busoff=%u ez_errpass=%u ez_warn=%u ez_tec=%u ez_rec=%u "
      "keya_busoff=%u keya_errpass=%u keya_warn=%u keya_tec=%u keya_rec=%u",
      state->auto_mode ? "AUTO" : "MANUAL", state->reverse ? "REV" : "FWD",
      (double)state->pedal_percent, state->estop_dashboard, state->estop_side,
      state->estop_rf, (double)state->vehicle_speed_kph, (double)state->motor_rpm,
      (double)state->motor_voltage_v, (double)state->motor_current_a,
      (double)state->motor_temperature_c, (unsigned long)state->brake_period_us,
      (unsigned long)state->brake_pulse_us,
      state->brake_state == HIL_BRAKE_ENGAGED ? "ENGAGED" :
      (state->brake_state == HIL_BRAKE_RELEASED ? "RELEASED" : "UNKNOWN"),
      state->brake_pwm_lost, (double)state->steering_position,
      (double)state->steering_target, (unsigned long)state->ez_rx_count,
      (unsigned long)state->ez_tx_count, (unsigned long)state->keya_rx_count,
      (unsigned long)state->keya_tx_count, state->ez_can_enabled,
      state->keya_can_enabled, state->ez_can_healthy, state->keya_can_healthy,
      state->ez_fault, state->keya_fault,
      state->ez_plant_enabled, (double)state->ez_target_rpm,
      state->ez_running, state->ez_speed_control,
      (double)state->ez_command_rpm, (double)state->ez_command_current_a,
      (unsigned long)state->ez_rx_age_ms, state->keya_motor_enabled,
      (unsigned long)state->keya_rx_age_ms,
      state->ez_bus_off, state->ez_error_passive, state->ez_warning,
      state->ez_tec, state->ez_rec,
      state->keya_bus_off, state->keya_error_passive, state->keya_warning,
      state->keya_tec, state->keya_rec);
    return true;
  }
  if ((count == 1u) && token_is(tokens[0], "HELP")) {
    (void)snprintf(reply, reply_size,
      "HELP SET MODE|GEAR|ESTOP|PEDAL|SPEED|RPM|VOLTAGE|CURRENT|MOTOR_TEMP|"
      "STEER_POS|EZ_CAN|KEYA_CAN|PLANT|EZ_SQUARE ...; FAULT EZ|KEYA [DROP_TX] ON|OFF; "
      "STATUS");
    return true;
  }

  if ((count == 3u) && token_is(tokens[0], "SET") && token_is(tokens[1], "MODE")) {
    if (token_is(tokens[2], "AUTO")) { state->auto_mode = true; success = true; }
    else if (token_is(tokens[2], "MANUAL")) { state->auto_mode = false; success = true; }
  } else if ((count == 3u) && token_is(tokens[0], "SET") && token_is(tokens[1], "GEAR")) {
    if (token_is(tokens[2], "REVERSE")) { state->reverse = true; success = true; }
    else if (token_is(tokens[2], "FORWARD")) { state->reverse = false; success = true; }
  } else if ((count == 4u) && token_is(tokens[0], "SET") && token_is(tokens[1], "ESTOP")) {
    bool *field = NULL;
    if (token_is(tokens[2], "DASH")) field = &state->estop_dashboard;
    else if (token_is(tokens[2], "SIDE")) field = &state->estop_side;
    else if (token_is(tokens[2], "RF")) field = &state->estop_rf;
    if ((field != NULL) && (token_is(tokens[3], "ON") || token_is(tokens[3], "OFF"))) {
      *field = token_is(tokens[3], "ON"); success = true;
    }
  } else if ((count == 3u) && token_is(tokens[0], "SET") &&
             (token_is(tokens[1], "EZ_CAN") || token_is(tokens[1], "KEYA_CAN")) &&
             (token_is(tokens[2], "ON") || token_is(tokens[2], "OFF"))) {
    bool enabled = token_is(tokens[2], "ON");
    if (token_is(tokens[1], "EZ_CAN")) state->ez_can_enabled = enabled;
    else state->keya_can_enabled = enabled;
    success = true;
  } else if ((count == 3u) && token_is(tokens[0], "SET") &&
             token_is(tokens[1], "EZ_SQUARE") &&
             (token_is(tokens[2], "ON") || token_is(tokens[2], "OFF"))) {
    state->ez_square_fault = token_is(tokens[2], "ON");
    success = true;
  } else if ((count == 3u) && token_is(tokens[0], "SET") &&
             token_is(tokens[1], "PLANT") &&
             (token_is(tokens[2], "ON") || token_is(tokens[2], "OFF"))) {
    /* Turning the plant off leaves the current motion in place so the operator
     * keeps whatever the loop reached and can inject from there. */
    state->ez_plant_enabled = token_is(tokens[2], "ON");
    success = true;
  } else if ((count == 3u) && token_is(tokens[0], "SET")) {
    float *field = NULL;
    float minimum = 0.0f, maximum = 0.0f;
    if (token_is(tokens[1], "PEDAL")) { field = &state->pedal_percent; maximum = 100.0f; }
    else if (token_is(tokens[1], "SPEED")) { field = &state->vehicle_speed_kph; minimum = HIL_SPEED_MIN_KPH; maximum = HIL_SPEED_MAX_KPH; }
    else if (token_is(tokens[1], "RPM")) { field = &state->motor_rpm; minimum = HIL_RPM_MIN; maximum = HIL_RPM_MAX; }
    else if (token_is(tokens[1], "VOLTAGE")) { field = &state->motor_voltage_v; minimum = HIL_VOLTAGE_MIN; maximum = HIL_VOLTAGE_MAX; }
    else if (token_is(tokens[1], "CURRENT")) { field = &state->motor_current_a; minimum = HIL_CURRENT_MIN; maximum = HIL_CURRENT_MAX; }
    else if (token_is(tokens[1], "MOTOR_TEMP")) { field = &state->motor_temperature_c; minimum = HIL_MOTOR_TEMP_MIN; maximum = HIL_MOTOR_TEMP_MAX; }
    else if (token_is(tokens[1], "STEER_POS")) { field = &state->steering_position; minimum = HIL_STEERING_MIN_DEG; maximum = HIL_STEERING_MAX_DEG; }
    if ((field != NULL) && parse_float(tokens[2], minimum, maximum, &value)) {
      *field = value;
      if (field == &state->steering_position) state->steering_target = value;
      else if (field == &state->vehicle_speed_kph) {
        const float speed_mps = value / 3.6f;
        state->motor_rpm = speed_mps * 60.0f * HIL_EZ_GEAR_RATIO /
            (HIL_PI_F * HIL_EZ_TIRE_DIAMETER_M);
      } else if (field == &state->motor_rpm) {
        HilLogic_SetMotionFromRpm(state, value);
      }
      success = true;
    }
  } else if ((count == 3u) && token_is(tokens[0], "FAULT") &&
             (token_is(tokens[1], "EZ") || token_is(tokens[1], "KEYA")) &&
             (token_is(tokens[2], "ON") || token_is(tokens[2], "OFF"))) {
    bool enabled = token_is(tokens[2], "ON");
    if (token_is(tokens[1], "EZ")) state->ez_fault = enabled;
    else state->keya_fault = enabled;
    success = true;
  } else if ((count == 4u) && token_is(tokens[0], "FAULT") &&
             token_is(tokens[2], "DROP_TX") &&
             (token_is(tokens[3], "ON") || token_is(tokens[3], "OFF"))) {
    bool enabled = token_is(tokens[3], "ON");
    if (token_is(tokens[1], "EZ")) { state->ez_drop_tx = enabled; success = true; }
    else if (token_is(tokens[1], "KEYA")) { state->keya_drop_tx = enabled; success = true; }
  }

  if (!success) {
    set_error(reply, reply_size, "invalid_command_or_range");
    return false;
  }
  (void)snprintf(reply, reply_size, "OK %s", original);
  return true;
}

void HilLogic_EZEncodePart1Feedback(const HilState_t *state,
                                    const HilEZFeedback_t *feedback,
                                    uint8_t data[8])
{
  uint16_t voltage = encode_u01(state->motor_voltage_v);
  uint16_t bus = encode_offset_01(feedback->bus_current_a);
  uint16_t phase = encode_offset_01(feedback->phase_current_a);
  float rpm = feedback->rpm;
  if (!isfinite(rpm)) rpm = 0.0f;
  if (rpm < -32000.0f) rpm = -32000.0f;
  if (rpm > 32000.0f) rpm = 32000.0f;
  uint16_t encoded_rpm = (uint16_t)lroundf(rpm + 32000.0f);
  data[0] = (uint8_t)voltage; data[1] = (uint8_t)(voltage >> 8);
  data[2] = (uint8_t)bus; data[3] = (uint8_t)(bus >> 8);
  data[4] = (uint8_t)phase; data[5] = (uint8_t)(phase >> 8);
  data[6] = (uint8_t)encoded_rpm; data[7] = (uint8_t)(encoded_rpm >> 8);
}

void HilLogic_EZEncodePart1(const HilState_t *state, uint8_t data[8])
{
  const HilEZFeedback_t feedback = {state->motor_current_a, state->motor_current_a,
                                    state->motor_rpm};
  HilLogic_EZEncodePart1Feedback(state, &feedback, data);
}

HilEZFeedback_t HilLogic_EZSquareFeedback(HilEZSquareFault_t *fault,
                                          const HilEZCommand_t *command,
                                          float plant_rpm, float measured_current_a)
{
  HilEZFeedback_t out = {measured_current_a, measured_current_a, plant_rpm};
  if (fault == NULL) return out;
  const bool torque = (command != NULL) && !command->handshake_ack &&
                      command->running && !command->speed_control;
  const float demand = torque ? command->phase_current_a : 0.0f;
  const bool drive = demand > 0.05f;
  const bool rolling = fabsf(plant_rpm) >= HIL_EZ_SQUARE_ROLLING_RPM;
  /* Trigger: reapply after 0 A, or a small held current, while rolling. */
  if ((fault->collapse_frame == 0u) && torque && rolling &&
      ((drive && !fault->previous_drive) || (drive && demand < HIL_EZ_SQUARE_STABLE_A))) {
    fault->collapse_frame = 1u;
    fault->collapses++;
  }
  fault->previous_drive = drive;
  if (torque) {
    /* Below the dead zone the bridge is off: exactly zero is reported. */
    const float reported = (fabsf(demand) < HIL_EZ_SQUARE_DEADZONE_A) ? 0.0f : demand;
    out.bus_current_a = out.phase_current_a = reported;
  }
  /* Logged shape: current/bus spike, RPM 0, sign flip, then recovery. */
  switch (fault->collapse_frame) {
    case 1u: out.phase_current_a = -120.0f; out.bus_current_a = 150.0f; break;
    case 2u: out.phase_current_a = 90.0f; out.rpm = 0.0f; break;
    case 3u: out.rpm = -0.25f * plant_rpm; break;
    default: break;
  }
  if (fault->collapse_frame != 0u) {
    fault->collapse_frame = (fault->collapse_frame >= HIL_EZ_SQUARE_COLLAPSE_FRAMES) ?
        0u : (uint8_t)(fault->collapse_frame + 1u);
  }
  return out;
}

void HilLogic_EZEncodePart2(const HilState_t *state, uint8_t life,
                            uint8_t data[8])
{
  float temperature = state->motor_temperature_c;
  if (temperature < -40.0f) temperature = -40.0f;
  if (temperature > 210.0f) temperature = 210.0f;
  const uint8_t encoded_temperature = (uint8_t)lroundf(temperature + 40.0f);
  const uint32_t fault_bits = state->ez_fault ? (1UL << 21) : 0u;
  data[0] = encoded_temperature;
  data[1] = encoded_temperature;
  data[2] = (state->ez_running ? 0x01u : 0x00u) |
            (state->ez_speed_control ? 0x02u : 0x00u);
  data[3] = (uint8_t)fault_bits;
  data[4] = (uint8_t)(fault_bits >> 8);
  data[5] = (uint8_t)(fault_bits >> 16);
  data[6] = 0u;
  data[7] = life;
}

bool HilLogic_IsFilled(const uint8_t data[8], uint8_t value)
{
  for (size_t i = 0u; i < 8u; ++i) if (data[i] != value) return false;
  return true;
}

static int32_t read_be32(const uint8_t *data)
{
  return (int32_t)(((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                   ((uint32_t)data[2] << 8) | (uint32_t)data[3]);
}

HilKeyaCommand_t HilLogic_KeyaDecode(const uint8_t data[8])
{
  HilKeyaCommand_t command = {HIL_KEYA_COMMAND_INVALID, 0};
  if ((data[0] == 0x23u) && (data[1] == 0x0Du) && (data[2] == 0x20u) && (data[3] == 0x01u)) command.type = HIL_KEYA_COMMAND_ENABLE;
  else if ((data[0] == 0x23u) && (data[1] == 0x0Cu) && (data[2] == 0x20u) && (data[3] == 0x01u)) command.type = HIL_KEYA_COMMAND_DISABLE;
  else if ((data[0] == 0x23u) && (data[1] == 0x02u) && (data[2] == 0x20u) && (data[3] == 0x01u)) {
    command.type = HIL_KEYA_COMMAND_POSITION;
    command.position_count = read_be32(&data[4]);
  } else if ((data[0] == 0x40u) && (data[2] == 0x21u)) {
    if ((data[1] == 0x04u) && (data[3] == 0x02u)) command.type = HIL_KEYA_QUERY_POSITION;
    else if ((data[1] == 0x03u) && (data[3] == 0x01u)) command.type = HIL_KEYA_QUERY_SPEED;
    else if ((data[1] == 0x00u) && (data[3] == 0x01u)) command.type = HIL_KEYA_QUERY_CURRENT;
    else if ((data[1] == 0x12u) && (data[3] == 0x01u)) command.type = HIL_KEYA_QUERY_FAULT;
    else if ((data[1] == 0x0Du) && (data[3] == 0x02u)) command.type = HIL_KEYA_QUERY_VOLTAGE;
    else if ((data[1] == 0x0Fu) && (data[3] == 0x01u)) command.type = HIL_KEYA_QUERY_TEMPERATURE;
  }
  return command;
}

int32_t HilLogic_KeyaDegreesToCount(float degrees)
{
  return (int32_t)lroundf(degrees * HIL_KEYA_COUNTS_PER_REV / 360.0f);
}

float HilLogic_KeyaCountToDegrees(int32_t count)
{
  return (float)count * 360.0f / HIL_KEYA_COUNTS_PER_REV;
}
