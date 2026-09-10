// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/RuntimeStats.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__RUNTIME_STATS__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__RUNTIME_STATS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'BOARD_STM_A'.
enum
{
  kaiev26_msgs__msg__RuntimeStats__BOARD_STM_A = 1
};

/// Constant 'BOARD_STM_B'.
enum
{
  kaiev26_msgs__msg__RuntimeStats__BOARD_STM_B = 2
};

// Include directives for member types
// Member 'tasks'
#include "kaiev26_msgs/msg/detail/task_runtime_stats__struct.h"

/// Struct defined in msg/RuntimeStats in the package kaiev26_msgs.
/**
  * Independent diagnostics: /stm_a/runtime_stats and /stm_b/runtime_stats, ~1 Hz,
  * best effort. No Header/string/sequence: fixed storage on the MCU.
 */
typedef struct kaiev26_msgs__msg__RuntimeStats
{
  uint8_t board_id;
  /// completed snapshot sequence; uint32 wrap
  uint32_t generation;
  uint32_t core_clock_hz;
  /// elapsed time between PerfMon aggregations
  uint32_t window_ms;
  /// board RTOS uptime at aggregation, not ROS time
  uint64_t uptime_ms;
  /// DWT timing source available
  bool timing_available;
  /// cumulative instrumentation errors; not safety faults
  uint32_t instrumentation_errors;
  /// FreeRTOS heap_4 current free bytes
  uint32_t free_heap_bytes;
  /// FreeRTOS heap_4 minimum-ever free bytes
  uint32_t minimum_free_heap_bytes;
  /// currently false on both boards
  bool cpu_load_valid;
  /// ; ignore when cpu_load_valid=false
  uint32_t cpu_load_permille;
  /// Slot i always has task_id=i, even if present=false. Counters/extrema are cumulative
  /// since boot, NOT reset each window. Compute rates from count delta / uptime delta
  /// on the same board/boot; publication intervals may differ from window_ms.
  kaiev26_msgs__msg__TaskRuntimeStats tasks[9];
  /// EXTI callback -> EStop task processing start; NOT physical-edge interrupt latency.
  /// Valid only on STM_B with timing available and at least one measured dispatch.
  /// STM_A always sends false/zero. Counters wrap at uint32; coalesced edges share the
  /// earliest pending callback timestamp. No sample means no latency evidence.
  bool estop_valid;
  uint32_t estop_callback_count;
  uint32_t estop_coalesced_count;
  uint32_t estop_dispatch_samples;
  /// cumulative observed maximum, integer microseconds
  uint32_t estop_dispatch_us_max;
} kaiev26_msgs__msg__RuntimeStats;

// Struct for a sequence of kaiev26_msgs__msg__RuntimeStats.
typedef struct kaiev26_msgs__msg__RuntimeStats__Sequence
{
  kaiev26_msgs__msg__RuntimeStats * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__RuntimeStats__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__RUNTIME_STATS__STRUCT_H_
