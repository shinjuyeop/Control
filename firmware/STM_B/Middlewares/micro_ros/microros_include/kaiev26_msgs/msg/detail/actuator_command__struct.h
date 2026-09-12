// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/ActuatorCommand.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__ACTUATOR_COMMAND__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__ACTUATOR_COMMAND__STRUCT_H_

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

/// Struct defined in msg/ActuatorCommand in the package kaiev26_msgs.
typedef struct kaiev26_msgs__msg__ActuatorCommand
{
  std_msgs__msg__Header header;
  /// 중앙 등가 Ackermann 조향각. 좌회전 양수
  float steering_target_rad;
  /// 실차 STM_A는 speed control만 지원한다. throttle/torque 모드는 제공하지 않는다
  float speed_target_mps;
  bool brake_engage;
} kaiev26_msgs__msg__ActuatorCommand;

// Struct for a sequence of kaiev26_msgs__msg__ActuatorCommand.
typedef struct kaiev26_msgs__msg__ActuatorCommand__Sequence
{
  kaiev26_msgs__msg__ActuatorCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__ActuatorCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__ACTUATOR_COMMAND__STRUCT_H_
