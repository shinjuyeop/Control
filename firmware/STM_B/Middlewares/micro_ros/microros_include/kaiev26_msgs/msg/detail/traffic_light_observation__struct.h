// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/TrafficLightObservation.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__TRAFFIC_LIGHT_OBSERVATION__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__TRAFFIC_LIGHT_OBSERVATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STATE_UNKNOWN'.
enum
{
  kaiev26_msgs__msg__TrafficLightObservation__STATE_UNKNOWN = 0
};

/// Constant 'STATE_RED'.
enum
{
  kaiev26_msgs__msg__TrafficLightObservation__STATE_RED = 1
};

/// Constant 'STATE_YELLOW'.
enum
{
  kaiev26_msgs__msg__TrafficLightObservation__STATE_YELLOW = 2
};

/// Constant 'STATE_GREEN'.
enum
{
  kaiev26_msgs__msg__TrafficLightObservation__STATE_GREEN = 3
};

/// Constant 'STATE_ARROW'.
enum
{
  kaiev26_msgs__msg__TrafficLightObservation__STATE_ARROW = 4
};

// Include directives for member types
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'source'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/TrafficLightObservation in the package kaiev26_msgs.
typedef struct kaiev26_msgs__msg__TrafficLightObservation
{
  uint32_t detection_id;
  uint32_t track_id;
  uint8_t state;
  /// header.frame_id 기준 위치, 미추정시 x/y/z NaN
  geometry_msgs__msg__Point position;
  /// 적색 score [0, 1], 색상 미판정시 NaN
  float red_score;
  /// 황색 score [0, 1], 색상 미판정시 NaN
  float yellow_score;
  /// 녹색 score [0, 1], 색상 미판정시 NaN
  float green_score;
  /// 화살표 score [0, 1], 색상 미판정시 NaN
  float arrow_score;
  /// 영상 감지/색상 신뢰도 [0, 1], 깊이 추정과 독립
  float confidence;
  rosidl_runtime_c__String source;
} kaiev26_msgs__msg__TrafficLightObservation;

// Struct for a sequence of kaiev26_msgs__msg__TrafficLightObservation.
typedef struct kaiev26_msgs__msg__TrafficLightObservation__Sequence
{
  kaiev26_msgs__msg__TrafficLightObservation * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__TrafficLightObservation__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__TRAFFIC_LIGHT_OBSERVATION__STRUCT_H_
