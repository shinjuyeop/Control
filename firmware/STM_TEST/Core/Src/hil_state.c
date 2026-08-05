#include "hil_state.h"

#include "cmsis_os2.h"
#include <string.h>

extern osMutexId_t HilStateMutexHandle;

static HilState_t s_hil_state;

void HilState_Init(void)
{
  memset(&s_hil_state, 0, sizeof(s_hil_state));
  s_hil_state.ez_can_enabled = true;
  s_hil_state.keya_can_enabled = true;
  s_hil_state.brake_pwm_lost = true;
  s_hil_state.brake_state = HIL_BRAKE_UNKNOWN;
}

HilState_t *HilState_Lock(uint32_t timeout_ticks)
{
  if ((HilStateMutexHandle == NULL) ||
      (osMutexAcquire(HilStateMutexHandle, timeout_ticks) != osOK)) {
    return NULL;
  }
  return &s_hil_state;
}

void HilState_Unlock(void)
{
  (void)osMutexRelease(HilStateMutexHandle);
}

bool HilState_GetSnapshot(HilState_t *snapshot, uint32_t timeout_ticks)
{
  if (snapshot == NULL) return false;
  HilState_t *state = HilState_Lock(timeout_ticks);
  if (state == NULL) return false;
  *snapshot = *state;
  HilState_Unlock();
  return true;
}
