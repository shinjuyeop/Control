#ifndef RUNTIME_METRICS_H
#define RUNTIME_METRICS_H

#include <stdbool.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "stm32g4xx.h"

typedef enum {
  RM_CONTROL, RM_INPUTS, RM_UROS, RM_RX, RM_TX,
  RM_BRAKE, RM_ESTOP, RM_DISPLAY, RM_PERFMON, RM_TASK_COUNT
} RuntimeTaskId;

typedef struct {
  uint64_t activations, completed, timed_samples, deadline_misses;
  uint64_t wall_cycles_total;
  uint32_t wall_cycles_max;
  int32_t jitter_ticks_min, jitter_ticks_max;
  uint64_t jitter_samples;
  uint32_t stack_free_words;
  uint64_t wall_us_total;
  uint32_t wall_us_max, wall_us_average;
  int64_t jitter_us_min, jitter_us_max;
  uint32_t present;
} RuntimeTaskSnapshot;

typedef struct {
  uint32_t callbacks, coalesced, dispatch_samples;
  uint32_t last_isr_cycle, last_task_cycle, dispatch_cycles_max;
  uint32_t dispatch_us_max;
} RuntimeEStopSnapshot;

typedef struct {
  uint32_t generation, core_clock_hz, window_ms;
  uint64_t uptime_ms;
  uint32_t free_heap_bytes, minimum_free_heap_bytes;
  uint32_t instrumentation_errors, timing_available;
  /* No scheduler/idle hooks installed: CPU load is explicitly unavailable. */
  uint32_t cpu_load_valid, cpu_load_permille;
  RuntimeTaskSnapshot tasks[RM_TASK_COUNT];
  RuntimeEStopSnapshot estop;
} RuntimeMetricsSnapshot;

/* One instance per task. All cycle intervals must fit in one CYCCNT wrap. */
typedef struct {
  uint32_t cycle;
  TickType_t tick;
} RuntimeActivation;

/* Init/register before scheduler start; remaining APIs are task-only except
 * Now/RecordEStopISR. IDs and handles are trusted firmware-owned constants. */
void RuntimeMetrics_Init(void);
void RuntimeMetrics_Register(RuntimeTaskId id, TaskHandle_t handle);
static inline uint32_t RuntimeMetrics_Now(void) { return DWT->CYCCNT; }
RuntimeActivation RuntimeMetrics_Begin(RuntimeTaskId id);
void RuntimeMetrics_End(RuntimeTaskId id, RuntimeActivation start);
void RuntimeMetrics_Release(RuntimeTaskId id, int32_t jitter_ticks);
void RuntimeMetrics_DeadlineMiss(RuntimeTaskId id);
void RuntimeMetrics_RecordEStopISR(uint32_t entry_cycle);
void RuntimeMetrics_EStopDispatch(uint32_t task_cycle);
/* Single writer: PerfMon, approximately 1 Hz. No ROS or allocation. */
void RuntimeMetrics_Aggregate(void);
/* Copy into caller-owned storage. False means not ready/concurrent publication;
 * retry on a later task iteration, never spin at higher priority than PerfMon.
 * On false, ignore out. Do not pass ISR stack storage or publish live internals. */
bool RuntimeMetrics_GetSnapshot(RuntimeMetricsSnapshot *out);

#endif
