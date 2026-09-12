// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/TaskRuntimeStats.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__TASK_RUNTIME_STATS__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__TASK_RUNTIME_STATS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'CONTROL'.
enum
{
  kaiev26_msgs__msg__TaskRuntimeStats__CONTROL = 0
};

/// Constant 'INPUTS'.
enum
{
  kaiev26_msgs__msg__TaskRuntimeStats__INPUTS = 1
};

/// Constant 'UROS'.
enum
{
  kaiev26_msgs__msg__TaskRuntimeStats__UROS = 2
};

/// Constant 'RX'.
enum
{
  kaiev26_msgs__msg__TaskRuntimeStats__RX = 3
};

/// Constant 'TX'.
enum
{
  kaiev26_msgs__msg__TaskRuntimeStats__TX = 4
};

/// Constant 'BRAKE'.
enum
{
  kaiev26_msgs__msg__TaskRuntimeStats__BRAKE = 5
};

/// Constant 'ESTOP'.
enum
{
  kaiev26_msgs__msg__TaskRuntimeStats__ESTOP = 6
};

/// Constant 'DISPLAY'.
enum
{
  kaiev26_msgs__msg__TaskRuntimeStats__DISPLAY = 7
};

/// Constant 'PERFMON'.
enum
{
  kaiev26_msgs__msg__TaskRuntimeStats__PERFMON = 8
};

/// Struct defined in msg/TaskRuntimeStats in the package kaiev26_msgs.
/**
  * Fixed-size MCU diagnostics. IDs match RuntimeTaskId in Control firmware.
 */
typedef struct kaiev26_msgs__msg__TaskRuntimeStats
{
  uint8_t task_id;
  /// false: task does not exist on this board
  bool present;
  /// cumulative since boot; includes in-progress activation
  uint64_t activations;
  /// cumulative completed activations
  uint64_t completed;
  /// valid wall-time samples; average denominator
  uint64_t timed_samples;
  /// cumulative observed activation wall time
  uint64_t wall_us_total;
  /// observed max, NOT formal WCET
  uint32_t wall_us_max;
  /// cumulative average; 0 if timed_samples=0
  uint32_t wall_us_average;
  /// 0: no scheduled-release jitter/deadline measurement
  uint64_t jitter_samples;
  /// actual activation - scheduled release
  int64_t jitter_us_min;
  /// 1 kHz RTOS tick gives 1000 us resolution
  int64_t jitter_us_max;
  /// cumulative; at most one per late activation
  uint64_t deadline_misses;
  /// minimum observed free stack since creation
  uint32_t stack_free_bytes;
} kaiev26_msgs__msg__TaskRuntimeStats;

// Struct for a sequence of kaiev26_msgs__msg__TaskRuntimeStats.
typedef struct kaiev26_msgs__msg__TaskRuntimeStats__Sequence
{
  kaiev26_msgs__msg__TaskRuntimeStats * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__TaskRuntimeStats__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__TASK_RUNTIME_STATS__STRUCT_H_
