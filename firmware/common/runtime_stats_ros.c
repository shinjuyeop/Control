#include "runtime_stats_ros.h"
#include "main.h"
#include <rclc/rclc.h>
#include <rcl/error_handling.h>

/* Public task IDs are a stable interface, independent of task presence. */
#define CHECK_ID(name) _Static_assert((int)RM_##name == (int)kaiev26_msgs__msg__TaskRuntimeStats__##name, "runtime task ID mismatch")
CHECK_ID(CONTROL); CHECK_ID(INPUTS); CHECK_ID(UROS); CHECK_ID(RX); CHECK_ID(TX);
CHECK_ID(BRAKE); CHECK_ID(ESTOP); CHECK_ID(DISPLAY); CHECK_ID(PERFMON);
#undef CHECK_ID
_Static_assert(RM_TASK_COUNT == 9, "RuntimeStats has nine fixed task slots");

void RuntimeStatsRos_Init(RuntimeStatsRos *state, rcl_node_t *node, uint8_t board_id)
{
  state->publisher = rcl_get_zero_initialized_publisher();
  state->board_id = board_id;
  state->last_attempt_ms = HAL_GetTick();
  const char *topic = board_id == kaiev26_msgs__msg__RuntimeStats__BOARD_STM_A ?
      "/stm_a/runtime_stats" : "/stm_b/runtime_stats";
  state->initialized = rclc_publisher_init_best_effort(
      &state->publisher, node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(kaiev26_msgs, msg, RuntimeStats), topic) == RCL_RET_OK;
  /* This optional entity must not prevent the existing control entities running.
   * Retry entity creation only with the existing agent reconnect lifecycle. */
  if (!state->initialized) rcl_reset_error();
}

void RuntimeStatsRos_Fini(RuntimeStatsRos *state, rcl_node_t *node)
{
  if (state->initialized) {
    const rcl_ret_t result = rcl_publisher_fini(&state->publisher, node);
    (void)result;
    state->initialized = false;
  }
}

void RuntimeStatsRos_PublishIfDue(RuntimeStatsRos *state)
{
  uint32_t now = HAL_GetTick();
  if (!state->initialized || (uint32_t)(now - state->last_attempt_ms) < 1000u)
    return;
  state->last_attempt_ms = now;
  if (!RuntimeMetrics_GetSnapshot(&state->snapshot)) return;

  const RuntimeMetricsSnapshot *s = &state->snapshot;
  kaiev26_msgs__msg__RuntimeStats *m = &state->message;
  m->board_id = state->board_id;
  m->generation = s->generation;
  m->core_clock_hz = s->core_clock_hz;
  m->window_ms = s->window_ms;
  m->uptime_ms = s->uptime_ms;
  m->timing_available = s->timing_available != 0u;
  m->instrumentation_errors = s->instrumentation_errors;
  m->free_heap_bytes = s->free_heap_bytes;
  m->minimum_free_heap_bytes = s->minimum_free_heap_bytes;
  m->cpu_load_valid = s->cpu_load_valid != 0u;
  m->cpu_load_permille = s->cpu_load_permille;
  for (unsigned i = 0; i < RM_TASK_COUNT; ++i) {
    const RuntimeTaskSnapshot *t = &s->tasks[i];
    kaiev26_msgs__msg__TaskRuntimeStats *out = &m->tasks[i];
    out->task_id = i;
    out->present = t->present != 0u;
    out->activations = t->activations;
    out->completed = t->completed;
    out->timed_samples = t->timed_samples;
    out->wall_us_total = t->wall_us_total;
    out->wall_us_max = t->wall_us_max;
    out->wall_us_average = t->wall_us_average;
    out->jitter_samples = t->jitter_samples;
    out->jitter_us_min = t->jitter_us_min;
    out->jitter_us_max = t->jitter_us_max;
    out->deadline_misses = t->deadline_misses;
    out->stack_free_bytes = t->stack_free_words * sizeof(StackType_t);
  }
  bool is_b = state->board_id == kaiev26_msgs__msg__RuntimeStats__BOARD_STM_B;
  m->estop_valid = is_b && m->timing_available && s->estop.dispatch_samples != 0u;
  m->estop_callback_count = is_b ? s->estop.callbacks : 0u;
  m->estop_coalesced_count = is_b ? s->estop.coalesced : 0u;
  m->estop_dispatch_samples = is_b ? s->estop.dispatch_samples : 0u;
  m->estop_dispatch_us_max = is_b ? s->estop.dispatch_us_max : 0u;
  const rcl_ret_t result = rcl_publish(&state->publisher, m, NULL);
  if (result != RCL_RET_OK) rcl_reset_error();
  /* At least one second after completion, even after a delayed transport call.
   * No diagnostic retry/catch-up loop and no safety/connection state changes. */
  state->last_attempt_ms = HAL_GetTick();
}
