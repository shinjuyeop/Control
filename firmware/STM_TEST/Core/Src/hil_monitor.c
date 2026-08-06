#include "hil_monitor.h"

#include "hil_logic.h"
#include "hil_state.h"
#include "cmsis_os2.h"
#include <stdbool.h>
#include <stdint.h>

#define HIL_MONITOR_PERIOD_MS 50u

extern TIM_HandleTypeDef htim3;

static volatile uint32_t s_capture_sequence;
static volatile uint32_t s_period_us;
static volatile uint32_t s_pulse_us;
static volatile uint32_t s_last_capture_tick;
static bool s_initialized;

void HilMonitor_Init(void)
{
  s_capture_sequence = 0u;
  s_period_us = 0u;
  s_pulse_us = 0u;
  s_last_capture_tick = HAL_GetTick();
  s_initialized =
      (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1) == HAL_OK) &&
      (HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2) == HAL_OK);
}

void HilMonitor_TIMCaptureFromISR(TIM_HandleTypeDef *htim)
{
  if ((htim->Instance != TIM3) || (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1)) return;
  ++s_capture_sequence;
  s_period_us = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
  s_pulse_us = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
  s_last_capture_tick = HAL_GetTick();
  ++s_capture_sequence;
}

void HilMonitor_Task(void)
{
  uint32_t next = osKernelGetTickCount();
  for (;;) {
    uint32_t before, after, period, pulse, last_tick;
    do {
      before = s_capture_sequence;
      period = s_period_us;
      pulse = s_pulse_us;
      last_tick = s_last_capture_tick;
      after = s_capture_sequence;
    } while ((before != after) || ((before & 1u) != 0u));

    const uint32_t now = HAL_GetTick();
    const bool lost = !s_initialized || (period == 0u) ||
        ((uint32_t)(now - last_tick) >= HIL_BRAKE_PWM_TIMEOUT_MS);
    const HilBrakeState_t classification = lost ? HIL_BRAKE_UNKNOWN :
        HilLogic_ClassifyBrakePulse(pulse);
    HilState_t *state = HilState_Lock(5u);
    if (state != NULL) {
      state->brake_period_us = period;
      state->brake_pulse_us = pulse;
      state->brake_pwm_lost = lost;
      state->brake_pwm_valid = !lost && (classification != HIL_BRAKE_UNKNOWN);
      state->brake_state = classification;
      state->brake_engaged = (classification == HIL_BRAKE_ENGAGED);
      HilState_Unlock();
    }

    next += HIL_MONITOR_PERIOD_MS;
    (void)osDelayUntil(next);
  }
}
