// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/SimulationGroundTruth.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__SIMULATION_GROUND_TRUTH__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__SIMULATION_GROUND_TRUTH__STRUCT_H_

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

/// Struct defined in msg/SimulationGroundTruth in the package kaiev26_msgs.
/**
  * Gazebo 전용 joint 참값. 실차에 대응 센서가 없으므로 공통 제어 계약이 아니며
  * 평가·디버그 용도로만 사용한다. 공통 제어 노드가 이 메시지에 의존하면 안 된다.
 */
typedef struct kaiev26_msgs__msg__SimulationGroundTruth
{
  std_msgs__msg__Header header;
  float front_left_steering_rad;
  float front_right_steering_rad;
  float rear_left_wheel_radps;
  float rear_right_wheel_radps;
} kaiev26_msgs__msg__SimulationGroundTruth;

// Struct for a sequence of kaiev26_msgs__msg__SimulationGroundTruth.
typedef struct kaiev26_msgs__msg__SimulationGroundTruth__Sequence
{
  kaiev26_msgs__msg__SimulationGroundTruth * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__SimulationGroundTruth__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__SIMULATION_GROUND_TRUTH__STRUCT_H_
