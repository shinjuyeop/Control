/* Native tests of the production module's arithmetic, publication and schedule.
 * RTOS tick/DWT registers are controlled here; no real-time claims on the host. */
#include <assert.h>
#include <stdio.h>
#include "runtime_periodic.h"

TestDwt test_dwt;
TestCoreDebug test_core_debug;
uint32_t SystemCoreClock = 170000000u;
static TickType_t tick;
static unsigned critical_depth;
static bool inspect_publication;
static unsigned rejected_reads;
static RuntimeMetricsSnapshot snapshot;

void test_critical_enter(void) { ++critical_depth; }
void test_critical_exit(void) { assert(critical_depth); --critical_depth; }
TickType_t xTaskGetTickCount(void) { return tick; }
TickType_t xTaskGetTickCountFromISR(void) { return tick; }
size_t xPortGetFreeHeapSize(void) { return 10000u; }
size_t xPortGetMinimumEverFreeHeapSize(void) { return 8000u; }
uint32_t uxTaskGetStackHighWaterMark(TaskHandle_t handle)
{
  assert(critical_depth == 0u && handle != NULL);
  return 128u;
}
void vTaskDelayUntil(TickType_t *previous, TickType_t period)
{
  *previous += period;
  if ((int32_t)(*previous - tick) > 0) tick = *previous;
}
void test_barrier(void)
{
  if (inspect_publication) {
    /* A higher-priority reader preempts an in-progress publication. It must
     * return immediately instead of waiting for the lower-priority writer. */
    assert(!RuntimeMetrics_GetSnapshot(&snapshot));
    ++rejected_reads;
  }
}

int main(void)
{
  assert(!RuntimeMetrics_GetSnapshot(&snapshot));
  RuntimeMetrics_Init();
  RuntimeMetrics_Register(RM_CONTROL, (TaskHandle_t)1);

  tick = 100u;
  test_dwt.CYCCNT = UINT32_MAX - 99u;
  RuntimeActivation a = RuntimeMetrics_Begin(RM_CONTROL);
  test_dwt.CYCCNT = 70u; /* 170 cycles across wrap = 1 us. */
  RuntimeMetrics_End(RM_CONTROL, a);
  inspect_publication = true;
  RuntimeMetrics_Aggregate();
  inspect_publication = false;
  assert(rejected_reads == 2u);
  assert(RuntimeMetrics_GetSnapshot(&snapshot));
  assert(snapshot.tasks[RM_CONTROL].wall_us_max == 1u);
  assert(snapshot.tasks[RM_CONTROL].wall_cycles_total == 170u);
  assert(snapshot.tasks[RM_CONTROL].stack_free_words == 128u);
  assert(snapshot.free_heap_bytes == 10000u);
  assert(!snapshot.cpu_load_valid);
  RuntimeMetricsSnapshot saved = snapshot;

  /* Overlong samples must not be interpreted as a short wrapped interval. */
  a = RuntimeMetrics_Begin(RM_CONTROL);
  tick += 30000u;
  test_dwt.CYCCNT += 170u;
  RuntimeMetrics_End(RM_CONTROL, a);
  RuntimeMetrics_Aggregate();
  assert(RuntimeMetrics_GetSnapshot(&snapshot));
  assert(snapshot.instrumentation_errors == 1u);
  assert(snapshot.tasks[RM_CONTROL].completed == 2u);
  assert(snapshot.tasks[RM_CONTROL].timed_samples == 1u);
  assert(saved.tasks[RM_CONTROL].completed == 1u);

  /* Phase and no-catch-up behavior, including TickType_t wrap. */
  tick = UINT32_MAX - 10u;
  RuntimePeriodic p = RuntimePeriodic_Init(20u);
  a = RuntimePeriodic_Begin(RM_TX, &p);
  tick += 2u;
  RuntimePeriodic_Wait(RM_TX, &p, a);
  assert(tick == 9u && p.release == tick);
  tick += 7u; /* delayed dispatch; next release follows actual start */
  a = RuntimePeriodic_Begin(RM_TX, &p);
  tick += 2u;
  RuntimePeriodic_Wait(RM_TX, &p, a);
  assert(tick == 36u);
  a = RuntimePeriodic_Begin(RM_TX, &p);
  tick += 55u;
  RuntimePeriodic_Wait(RM_TX, &p, a);
  assert(tick == 111u); /* full period after overrun completion */
  a = RuntimePeriodic_Begin(RM_TX, &p);
  tick += 20u; /* exact deadline also must not replay a release */
  RuntimePeriodic_Wait(RM_TX, &p, a);
  assert(tick == 151u);

  tick = 1000u;
  RuntimeMetrics_RecordEStopISR(UINT32_MAX - 99u);
  RuntimeMetrics_RecordEStopISR(10u);
  RuntimeMetrics_EStopDispatch(240u); /* earliest pending -> 340 cycles */
  RuntimeMetrics_Aggregate();
  assert(RuntimeMetrics_GetSnapshot(&snapshot));
  assert(snapshot.estop.callbacks == 2u && snapshot.estop.coalesced == 1u);
  assert(snapshot.estop.dispatch_samples == 1u);
  assert(snapshot.estop.dispatch_us_max == 2u);
  assert(snapshot.tasks[RM_TX].deadline_misses == 2u);
  assert(snapshot.tasks[RM_TX].jitter_ticks_max == 7);

  /* A callback newer than the captured dispatch is retained for next pass. */
  RuntimeMetrics_RecordEStopISR(1000u);
  RuntimeMetrics_EStopDispatch(900u);
  RuntimeMetrics_EStopDispatch(1510u);
  RuntimeMetrics_Aggregate();
  assert(RuntimeMetrics_GetSnapshot(&snapshot));
  assert(snapshot.estop.dispatch_samples == 2u);
  assert(snapshot.estop.dispatch_us_max == 3u);
  RuntimeMetrics_Release(RM_INPUTS, 3000000);
  RuntimeMetrics_Aggregate();
  assert(RuntimeMetrics_GetSnapshot(&snapshot));
  assert(snapshot.tasks[RM_INPUTS].jitter_us_max == 3000000000LL);
  assert(critical_depth == 0u);
  puts("runtime metrics and periodic scheduling tests passed");
  return 0;
}
