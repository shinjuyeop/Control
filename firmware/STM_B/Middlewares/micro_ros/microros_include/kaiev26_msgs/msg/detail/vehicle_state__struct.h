// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/VehicleState.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__VEHICLE_STATE__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__VEHICLE_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'GEAR_FORWARD'.
enum
{
  kaiev26_msgs__msg__VehicleState__GEAR_FORWARD = 0
};

/// Constant 'GEAR_REVERSE'.
enum
{
  kaiev26_msgs__msg__VehicleState__GEAR_REVERSE = 1
};

/// Constant 'MODE_MANUAL'.
/**
  * 실차 모드 스위치 규약과 동일
 */
enum
{
  kaiev26_msgs__msg__VehicleState__MODE_MANUAL = 0
};

/// Constant 'MODE_AUTO'.
enum
{
  kaiev26_msgs__msg__VehicleState__MODE_AUTO = 1
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/VehicleState in the package kaiev26_msgs.
/**
  * D-09: 실차가 실제로 제공 가능한 최소 공통 상태만 담는다.
  * sim 전용 joint 참값은 SimulationGroundTruth(/simulation/ground_truth)로 분리한다.
  * 상세 액추에이터 진단은 별도 status 메시지로 제공한다.
 */
typedef struct kaiev26_msgs__msg__VehicleState
{
  std_msgs__msg__Header header;
  float speed_mps;
  /// 중앙 등가 조향각 추정치. 실차는 motor encoder 기반 추정만 가능하다.
  float steering_rad;
  uint8_t gear;
  uint8_t mode;
  bool estop_active;
} kaiev26_msgs__msg__VehicleState;

// Struct for a sequence of kaiev26_msgs__msg__VehicleState.
typedef struct kaiev26_msgs__msg__VehicleState__Sequence
{
  kaiev26_msgs__msg__VehicleState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__VehicleState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__VEHICLE_STATE__STRUCT_H_
