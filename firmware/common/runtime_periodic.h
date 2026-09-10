#ifndef RUNTIME_PERIODIC_H
#define RUNTIME_PERIODIC_H

#include "runtime_metrics.h"

typedef struct {
  TickType_t release, actual, period;
} RuntimePeriodic;

static inline RuntimePeriodic RuntimePeriodic_Init(TickType_t period)
{
  RuntimePeriodic p = {xTaskGetTickCount(), 0u, period};
  return p;
}

static inline RuntimeActivation RuntimePeriodic_Begin(
    RuntimeTaskId id, RuntimePeriodic *p)
{
  RuntimeActivation start = RuntimeMetrics_Begin(id);
  p->actual = start.tick;
  RuntimeMetrics_Release(id, (int32_t)(p->actual - p->release));
  return start;
}

static inline void RuntimePeriodic_Wait(
    RuntimeTaskId id, RuntimePeriodic *p, RuntimeActivation start)
{
  RuntimeMetrics_End(id, start);
  TickType_t now = xTaskGetTickCount();
  if ((int32_t)(now - p->release) >= (int32_t)p->period)
    RuntimeMetrics_DeadlineMiss(id);
  /* Anchor to actual activation after a delayed wake. If work itself overran,
   * wait a full period from completion. Never replay missed releases. */
  TickType_t anchor = p->actual;
  if ((TickType_t)(now - anchor) >= p->period) anchor = now;
  vTaskDelayUntil(&anchor, p->period);
  p->release = anchor;
}

#endif
