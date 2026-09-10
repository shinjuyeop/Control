#include "runtime_metrics.h"

static RuntimeTaskSnapshot s_tasks[RM_TASK_COUNT];
static TaskHandle_t s_handles[RM_TASK_COUNT];
static volatile RuntimeEStopSnapshot s_estop;
static volatile uint32_t s_pending, s_first_isr_cycle;
static volatile TickType_t s_first_isr_tick;
static uint32_t s_errors, s_timing_available, s_clock_hz;
static TickType_t s_wrap_limit_ticks, s_last_snapshot_tick;
static uint64_t s_uptime_ticks;
/* Static storage keeps the 256-word PerfMon stack independent of snapshot size. */
static RuntimeMetricsSnapshot s_staging;
static volatile RuntimeMetricsSnapshot s_published;
static volatile uint32_t s_publication;

void RuntimeMetrics_Init(void)
{
  s_clock_hz = SystemCoreClock;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  uint32_t before = RuntimeMetrics_Now();
  __NOP(); __NOP(); __NOP(); __NOP();
  s_timing_available = (RuntimeMetrics_Now() != before);
  /* Conservative tick guard also rejects intervals spanning multiple wraps.
   * A tick of uncertainty is excluded at the boundary. No division in hot paths. */
  s_wrap_limit_ticks = (TickType_t)(UINT32_MAX /
      (s_clock_hz / configTICK_RATE_HZ)) - 1u;
  if (!s_timing_available) s_errors++;
}

void RuntimeMetrics_Register(RuntimeTaskId id, TaskHandle_t handle)
{
  s_handles[id] = handle;
  s_tasks[id].present = (handle != NULL);
  if (handle == NULL) s_errors++;
}

RuntimeActivation RuntimeMetrics_Begin(RuntimeTaskId id)
{
  RuntimeActivation start = {RuntimeMetrics_Now(), xTaskGetTickCount()};
  taskENTER_CRITICAL();
  s_tasks[id].activations++;
  taskEXIT_CRITICAL();
  return start;
}

void RuntimeMetrics_End(RuntimeTaskId id, RuntimeActivation start)
{
  uint32_t elapsed = RuntimeMetrics_Now() - start.cycle;
  TickType_t ticks = xTaskGetTickCount() - start.tick;
  taskENTER_CRITICAL();
  RuntimeTaskSnapshot *task = &s_tasks[id];
  task->completed++;
  if (s_timing_available && ticks < s_wrap_limit_ticks) {
    task->timed_samples++;
    task->wall_cycles_total += elapsed;
    if (elapsed > task->wall_cycles_max) task->wall_cycles_max = elapsed;
  } else {
    s_errors++;
  }
  taskEXIT_CRITICAL();
}

void RuntimeMetrics_Release(RuntimeTaskId id, int32_t jitter_ticks)
{
  taskENTER_CRITICAL();
  RuntimeTaskSnapshot *task = &s_tasks[id];
  if (task->jitter_samples == 0u || jitter_ticks < task->jitter_ticks_min)
    task->jitter_ticks_min = jitter_ticks;
  if (task->jitter_samples == 0u || jitter_ticks > task->jitter_ticks_max)
    task->jitter_ticks_max = jitter_ticks;
  task->jitter_samples++;
  taskEXIT_CRITICAL();
}

void RuntimeMetrics_DeadlineMiss(RuntimeTaskId id)
{
  taskENTER_CRITICAL();
  s_tasks[id].deadline_misses++;
  taskEXIT_CRITICAL();
}

void RuntimeMetrics_RecordEStopISR(uint32_t entry_cycle)
{
  /* EXTI IRQs all have priority 5, cannot nest each other, and are masked by
   * taskENTER_CRITICAL in the consuming task. Preserve earliest pending edge. */
  s_estop.callbacks++;
  s_estop.last_isr_cycle = entry_cycle;
  if (s_pending == 0u) {
    s_first_isr_cycle = entry_cycle;
    s_first_isr_tick = xTaskGetTickCountFromISR();
    s_pending = 1u;
  } else {
    s_estop.coalesced++;
  }
}

void RuntimeMetrics_EStopDispatch(uint32_t task_cycle)
{
  taskENTER_CRITICAL();
  s_estop.last_task_cycle = task_cycle;
  if (s_pending != 0u) {
    uint32_t elapsed = task_cycle - s_first_isr_cycle;
    /* An IRQ arriving after task_cycle belongs to the next processing pass. */
    if ((int32_t)elapsed >= 0) {
      if (s_timing_available &&
          (xTaskGetTickCount() - s_first_isr_tick) < s_wrap_limit_ticks / 2u) {
        s_estop.dispatch_samples++;
        if (elapsed > s_estop.dispatch_cycles_max)
          s_estop.dispatch_cycles_max = elapsed;
      } else {
        s_errors++;
      }
      s_pending = 0u;
    } else if ((xTaskGetTickCount() - s_first_isr_tick) >= s_wrap_limit_ticks / 2u) {
      s_errors++;
      s_pending = 0u;
    }
  }
  taskEXIT_CRITICAL();
}

static uint64_t cycles_to_us(uint64_t cycles)
{
  /* Quotient first avoids overflowing the lifetime uint64_t cycle total. */
  return (cycles / s_clock_hz) * 1000000u +
      (cycles % s_clock_hz) * 1000000u / s_clock_hz;
}

void RuntimeMetrics_Aggregate(void)
{
  TickType_t now = xTaskGetTickCount();
  TickType_t elapsed = now - s_last_snapshot_tick;
  s_last_snapshot_tick = now;
  s_uptime_ticks += elapsed;
  s_staging.generation++;
  s_staging.core_clock_hz = s_clock_hz;
  s_staging.window_ms = (uint64_t)elapsed * 1000u / configTICK_RATE_HZ;
  s_staging.uptime_ms = s_uptime_ticks * 1000u / configTICK_RATE_HZ;
  s_staging.timing_available = s_timing_available;
  s_staging.free_heap_bytes = xPortGetFreeHeapSize();
  s_staging.minimum_free_heap_bytes = xPortGetMinimumEverFreeHeapSize();
  for (unsigned id = 0; id < RM_TASK_COUNT; ++id) {
    RuntimeTaskSnapshot *task = &s_staging.tasks[id];
    /* Only one raw record per critical section. Stack scans and conversions
     * run with interrupts/scheduling enabled, no priority-blocking mutex. */
    taskENTER_CRITICAL();
    *task = s_tasks[id];
    taskEXIT_CRITICAL();
    if (s_handles[id] != NULL)
      task->stack_free_words = uxTaskGetStackHighWaterMark(s_handles[id]);
    task->wall_us_total = cycles_to_us(task->wall_cycles_total);
    task->wall_us_max = (uint32_t)cycles_to_us(task->wall_cycles_max);
    task->wall_us_average = task->timed_samples == 0u ? 0u :
        (uint32_t)cycles_to_us(task->wall_cycles_total / task->timed_samples);
    task->jitter_us_min = (int64_t)task->jitter_ticks_min * 1000000 / configTICK_RATE_HZ;
    task->jitter_us_max = (int64_t)task->jitter_ticks_max * 1000000 / configTICK_RATE_HZ;
  }
  taskENTER_CRITICAL();
  s_staging.estop = s_estop;
  s_staging.instrumentation_errors = s_errors;
  taskEXIT_CRITICAL();
  s_staging.estop.dispatch_us_max =
      (uint32_t)cycles_to_us(s_staging.estop.dispatch_cycles_max);
  /* A preempting reader fails immediately while publication is in progress.
   * Volatile accesses + DMB delimit publication on these single-core MCUs. */
  s_publication++;
  __DMB();
  s_published = s_staging;
  __DMB();
  s_publication++;
}

bool RuntimeMetrics_GetSnapshot(RuntimeMetricsSnapshot *out)
{
  uint32_t generation = s_publication;
  if (generation == 0u || (generation & 1u) != 0u) return false;
  __DMB();
  *out = s_published;
  __DMB();
  return s_publication == generation;
}
