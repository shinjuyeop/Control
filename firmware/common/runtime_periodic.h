#ifndef RUNTIME_PERIODIC_H
#define RUNTIME_PERIODIC_H

#include "runtime_metrics.h"

typedef struct {
  TickType_t release, period;
} RuntimePeriodic;

static inline RuntimePeriodic RuntimePeriodic_Init(TickType_t period)
{
  RuntimePeriodic p = {xTaskGetTickCount(), period};
  return p;
}

static inline RuntimeActivation RuntimePeriodic_Begin(
    RuntimeTaskId id, RuntimePeriodic *p)
{
  RuntimeActivation start = RuntimeMetrics_Begin(id);
  RuntimeMetrics_Release(id, (int32_t)(start.tick - p->release));
  return start;
}

static inline void RuntimePeriodic_Wait(
    RuntimeTaskId id, RuntimePeriodic *p, RuntimeActivation start)
{
  RuntimeMetrics_End(id, start);
  TickType_t now = xTaskGetTickCount();
  TickType_t elapsed = now - p->release;
  if (elapsed >= p->period)
    RuntimeMetrics_DeadlineMiss(id);
  /* Keep the initial phase. Skip releases at/before completion in one step;
   * delay-until adds one more period to select the first future release.
   * Unsigned tick subtraction/addition handles wrap. Period is nonzero and
   * release-to-completion stays within the 32-bit tick half range, as for jitter. */
  p->release += (elapsed / p->period) * p->period;
  vTaskDelayUntil(&p->release, p->period);
}

#endif
