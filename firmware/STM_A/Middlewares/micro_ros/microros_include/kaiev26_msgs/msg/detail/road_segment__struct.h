// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/RoadSegment.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__ROAD_SEGMENT__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__ROAD_SEGMENT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'TYPE_WHSOL'.
/**
  * 흰색 실선
 */
enum
{
  kaiev26_msgs__msg__RoadSegment__TYPE_WHSOL = 1
};

/// Constant 'TYPE_WHDOT'.
/**
  * 흰색 점선
 */
enum
{
  kaiev26_msgs__msg__RoadSegment__TYPE_WHDOT = 2
};

/// Constant 'TYPE_YESOL'.
/**
  * 노란색 실선
 */
enum
{
  kaiev26_msgs__msg__RoadSegment__TYPE_YESOL = 3
};

/// Constant 'TYPE_YEDOT'.
/**
  * 노란색 점선
 */
enum
{
  kaiev26_msgs__msg__RoadSegment__TYPE_YEDOT = 4
};

/// Constant 'TYPE_STOP_LINE'.
/**
  * 정지선
 */
enum
{
  kaiev26_msgs__msg__RoadSegment__TYPE_STOP_LINE = 5
};

/// Constant 'TYPE_CROSSWALK'.
/**
  * 횡단보도
 */
enum
{
  kaiev26_msgs__msg__RoadSegment__TYPE_CROSSWALK = 6
};

/// Constant 'TYPE_BLSOL'.
/**
  * 파란색 실선
 */
enum
{
  kaiev26_msgs__msg__RoadSegment__TYPE_BLSOL = 7
};

/// Constant 'TYPE_BLDOT'.
/**
  * 파란색 점선
 */
enum
{
  kaiev26_msgs__msg__RoadSegment__TYPE_BLDOT = 8
};

// Include directives for member types
// Member 'points'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'source'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/RoadSegment in the package kaiev26_msgs.
typedef struct kaiev26_msgs__msg__RoadSegment
{
  uint32_t detection_id;
  uint32_t track_id;
  uint8_t type;
  geometry_msgs__msg__Point__Sequence points;
  float confidence;
  rosidl_runtime_c__String source;
} kaiev26_msgs__msg__RoadSegment;

// Struct for a sequence of kaiev26_msgs__msg__RoadSegment.
typedef struct kaiev26_msgs__msg__RoadSegment__Sequence
{
  kaiev26_msgs__msg__RoadSegment * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__RoadSegment__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__ROAD_SEGMENT__STRUCT_H_
