// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/BrakeActuatorStatus.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__BRAKE_ACTUATOR_STATUS__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__BRAKE_ACTUATOR_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STOP_REASON_SERVICE_BRAKE'.
enum
{
  kaiev26_msgs__msg__BrakeActuatorStatus__STOP_REASON_SERVICE_BRAKE = 1
};

/// Constant 'STOP_REASON_AEB'.
enum
{
  kaiev26_msgs__msg__BrakeActuatorStatus__STOP_REASON_AEB = 2
};

/// Constant 'STOP_REASON_ESTOP'.
enum
{
  kaiev26_msgs__msg__BrakeActuatorStatus__STOP_REASON_ESTOP = 4
};

/// Constant 'STOP_REASON_COMMAND_INVALID'.
enum
{
  kaiev26_msgs__msg__BrakeActuatorStatus__STOP_REASON_COMMAND_INVALID = 8
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/BrakeActuatorStatus in the package kaiev26_msgs.
/**
  * 브레이크 진단 status (D-10·D-11·D-13).
  * 실차 브레이크 서보는 피드백 신호선 자체가 없어 actual은 항구적 미지원 —
  * commanded만 제공하고 actual_valid=false 고정. sim도 계약을 따라 동일하게 둔다.
  * 일반 제동과 안전 정지 원인은 bitmask로 구분한다(D-04).
 */
typedef struct kaiev26_msgs__msg__BrakeActuatorStatus
{
  std_msgs__msg__Header header;
  bool brake_commanded;
  uint8_t stop_reason_bits;
  bool actual_valid;
  float command_age_ms;
} kaiev26_msgs__msg__BrakeActuatorStatus;

// Struct for a sequence of kaiev26_msgs__msg__BrakeActuatorStatus.
typedef struct kaiev26_msgs__msg__BrakeActuatorStatus__Sequence
{
  kaiev26_msgs__msg__BrakeActuatorStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__BrakeActuatorStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__BRAKE_ACTUATOR_STATUS__STRUCT_H_
