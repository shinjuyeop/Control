#ifndef HIL_LOGIC_H
#define HIL_LOGIC_H

#include "hil_state.h"
#include <stddef.h>
#include <stdint.h>

#define HIL_PEDAL_ADC_MIN                 560u
#define HIL_PEDAL_ADC_MAX                3000u
#define HIL_DAC_MAX_COUNT                4095u

#define HIL_BRAKE_ENGAGED_PULSE_US       1194u
#define HIL_BRAKE_RELEASED_PULSE_US      1388u
#define HIL_BRAKE_PULSE_TOLERANCE_US     50u
#define HIL_BRAKE_PWM_TIMEOUT_MS         100u

#define HIL_KEYA_COUNTS_PER_REV          10000.0f
#define HIL_KEYA_SIM_RATE_DEG_PER_SEC    90.0f

/* EZkontrol plant. First-order lag only: one time constant, no vehicle mass or
 * road load. It exists so a speed command reaches the motor RPM that STM_A
 * reads back, not to reproduce real acceleration. */
#define HIL_EZ_PLANT_TAU_S               0.60f
/* Torque control commands a phase current, not an RPM. The RPM field is a limit
 * in that mode, so scale it by the current as a fraction of the configured
 * maximum phase current. STM_A maps MANUAL pedal 0..100% to 0..480 A. */
#define HIL_EZ_PLANT_TORQUE_FULL_SCALE_A 480.0f
/* No command frame for this long means STM_A is gone or was reset. The emulator
 * returns to WAIT_HANDSHAKE so a restarted STM_A can hand shake again. */
#define HIL_EZ_RX_TIMEOUT_MS             500u

typedef enum {
  HIL_KEYA_COMMAND_INVALID = 0,
  HIL_KEYA_COMMAND_ENABLE,
  HIL_KEYA_COMMAND_DISABLE,
  HIL_KEYA_COMMAND_POSITION,
  HIL_KEYA_QUERY_POSITION,
  HIL_KEYA_QUERY_SPEED,
  HIL_KEYA_QUERY_CURRENT,
  HIL_KEYA_QUERY_FAULT,
  HIL_KEYA_QUERY_VOLTAGE,
  HIL_KEYA_QUERY_TEMPERATURE
} HilKeyaCommandType_t;

typedef struct {
  HilKeyaCommandType_t type;
  int32_t position_count;
} HilKeyaCommand_t;

uint16_t HilLogic_PedalPercentToDac(float percent);
HilBrakeState_t HilLogic_ClassifyBrakePulse(uint32_t pulse_us);
float HilLogic_Approach(float current, float target, float max_step);

bool HilLogic_ProcessCommand(HilState_t *state, char *line,
                             char *reply, size_t reply_size);

void HilLogic_EZEncodePart1(const HilState_t *state, uint8_t data[8]);
void HilLogic_EZEncodePart2(const HilState_t *state, uint8_t life,
                            uint8_t data[8]);
bool HilLogic_IsFilled(const uint8_t data[8], uint8_t value);

typedef struct {
  bool handshake_ack;
  bool running;
  bool speed_control;
  /* Commanded RPM in speed control, RPM limit in torque control. */
  float rpm_field;
  float phase_current_a;
} HilEZCommand_t;

HilEZCommand_t HilLogic_EZDecodeCommand(const uint8_t data[8]);
float HilLogic_EZPlantTargetRpm(const HilEZCommand_t *command);
float HilLogic_PlantStepRpm(float current_rpm, float target_rpm,
                            float dt_s, float tau_s);
void HilLogic_SetMotionFromRpm(HilState_t *state, float rpm);

HilKeyaCommand_t HilLogic_KeyaDecode(const uint8_t data[8]);
int32_t HilLogic_KeyaDegreesToCount(float degrees);
float HilLogic_KeyaCountToDegrees(int32_t count);

#endif /* HIL_LOGIC_H */
