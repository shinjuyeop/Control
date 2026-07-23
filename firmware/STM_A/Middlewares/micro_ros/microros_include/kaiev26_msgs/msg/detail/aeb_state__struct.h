// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/AEBState.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__AEB_STATE__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__AEB_STATE__STRUCT_H_

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
// Member 'state'
// Member 'reason'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/AEBState in the package kaiev26_msgs.
typedef struct kaiev26_msgs__msg__AEBState
{
  std_msgs__msg__Header header;
  bool active;
  rosidl_runtime_c__String state;
  rosidl_runtime_c__String reason;
  float front_distance;
  float ttc;
  float object_data_age_ms;
} kaiev26_msgs__msg__AEBState;

// Struct for a sequence of kaiev26_msgs__msg__AEBState.
typedef struct kaiev26_msgs__msg__AEBState__Sequence
{
  kaiev26_msgs__msg__AEBState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__AEBState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__AEB_STATE__STRUCT_H_
