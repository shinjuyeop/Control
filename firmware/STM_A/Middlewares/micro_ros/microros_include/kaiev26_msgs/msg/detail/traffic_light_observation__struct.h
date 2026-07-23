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
  geometry_msgs__msg__Point position;
  float red_score;
  float yellow_score;
  float green_score;
  float arrow_score;
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
