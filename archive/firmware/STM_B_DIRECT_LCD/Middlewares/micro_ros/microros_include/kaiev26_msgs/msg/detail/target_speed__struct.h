// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/TargetSpeed.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__TARGET_SPEED__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__TARGET_SPEED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'source_behavior'
// Member 'constraints'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/TargetSpeed in the package kaiev26_msgs.
typedef struct kaiev26_msgs__msg__TargetSpeed
{
  std_msgs__msg__Header header;
  float target_speed_mps;
  float speed_limit_mps;
  bool need_stop;
  float stop_target_distance;
  rosidl_runtime_c__String source_behavior;
  rosidl_runtime_c__String__Sequence constraints;
} kaiev26_msgs__msg__TargetSpeed;

// Struct for a sequence of kaiev26_msgs__msg__TargetSpeed.
typedef struct kaiev26_msgs__msg__TargetSpeed__Sequence
{
  kaiev26_msgs__msg__TargetSpeed * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__TargetSpeed__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__TARGET_SPEED__STRUCT_H_
