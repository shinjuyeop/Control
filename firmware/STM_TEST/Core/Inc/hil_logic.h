#ifndef HIL_LOGIC_H
#define HIL_LOGIC_H

#include "hil_state.h"
#include <stddef.h>
#include <stdint.h>

#define HIL_PEDAL_ADC_MIN                 560u
#define HIL_PEDAL_ADC_MAX                3100u
#define HIL_DAC_MAX_COUNT                4095u

#define HIL_BRAKE_ENGAGED_PULSE_US       1194u
#define HIL_BRAKE_RELEASED_PULSE_US      1388u
#define HIL_BRAKE_PULSE_TOLERANCE_US     50u
#define HIL_BRAKE_PWM_TIMEOUT_MS         100u

#define HIL_KEYA_COUNTS_PER_REV          10000.0f
#define HIL_KEYA_SIM_RATE_DEG_PER_SEC    90.0f

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

HilKeyaCommand_t HilLogic_KeyaDecode(const uint8_t data[8]);
int32_t HilLogic_KeyaDegreesToCount(float degrees);
float HilLogic_KeyaCountToDegrees(int32_t count);

#endif /* HIL_LOGIC_H */
