// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/BehaviorDecision.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__BEHAVIOR_DECISION__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__BEHAVIOR_DECISION__STRUCT_H_

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
// Member 'safety_state'
// Member 'active_behavior'
// Member 'fsm_state'
// Member 'selected_reason'
// Member 'path_request'
// Member 'constraints'
// Member 'risk_level'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/BehaviorDecision in the package kaiev26_msgs.
typedef struct kaiev26_msgs__msg__BehaviorDecision
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__String safety_state;
  rosidl_runtime_c__String active_behavior;
  rosidl_runtime_c__String fsm_state;
  rosidl_runtime_c__String selected_reason;
  bool need_stop;
  float target_speed_limit_mps;
  rosidl_runtime_c__String path_request;
  float stop_target_distance;
  rosidl_runtime_c__String__Sequence constraints;
  rosidl_runtime_c__String risk_level;
} kaiev26_msgs__msg__BehaviorDecision;

// Struct for a sequence of kaiev26_msgs__msg__BehaviorDecision.
typedef struct kaiev26_msgs__msg__BehaviorDecision__Sequence
{
  kaiev26_msgs__msg__BehaviorDecision * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__BehaviorDecision__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__BEHAVIOR_DECISION__STRUCT_H_
