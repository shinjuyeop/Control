// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/RoadFeature2D.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__ROAD_FEATURE2_D__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__ROAD_FEATURE2_D__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'KIND_UNKNOWN'.
/**
  * 미분류, ready payload에서는 사용 금지
 */
enum
{
  kaiev26_msgs__msg__RoadFeature2D__KIND_UNKNOWN = 0
};

/// Constant 'KIND_LANE'.
/**
  * 차선 경계 polyline
 */
enum
{
  kaiev26_msgs__msg__RoadFeature2D__KIND_LANE = 1
};

/// Constant 'KIND_STOP_LINE'.
/**
  * 정지선 polyline
 */
enum
{
  kaiev26_msgs__msg__RoadFeature2D__KIND_STOP_LINE = 2
};

/// Constant 'KIND_CROSSWALK'.
/**
  * 횡단보도 polygon
 */
enum
{
  kaiev26_msgs__msg__RoadFeature2D__KIND_CROSSWALK = 3
};

/// Constant 'COLOR_UNKNOWN'.
/**
  * 색상 미분류
 */
enum
{
  kaiev26_msgs__msg__RoadFeature2D__COLOR_UNKNOWN = 0
};

/// Constant 'COLOR_WHITE'.
/**
  * 흰색
 */
enum
{
  kaiev26_msgs__msg__RoadFeature2D__COLOR_WHITE = 1
};

/// Constant 'COLOR_YELLOW'.
/**
  * 노란색
 */
enum
{
  kaiev26_msgs__msg__RoadFeature2D__COLOR_YELLOW = 2
};

/// Constant 'COLOR_BLUE'.
/**
  * 파란색
 */
enum
{
  kaiev26_msgs__msg__RoadFeature2D__COLOR_BLUE = 3
};

/// Constant 'STYLE_UNKNOWN'.
/**
  * 형태 미분류 또는 해당 없음
 */
enum
{
  kaiev26_msgs__msg__RoadFeature2D__STYLE_UNKNOWN = 0
};

/// Constant 'STYLE_SOLID'.
/**
  * 실선
 */
enum
{
  kaiev26_msgs__msg__RoadFeature2D__STYLE_SOLID = 1
};

/// Constant 'STYLE_DOTTED'.
/**
  * 점선
 */
enum
{
  kaiev26_msgs__msg__RoadFeature2D__STYLE_DOTTED = 2
};

// Include directives for member types
// Member 'points'
#include "geometry_msgs/msg/detail/point32__struct.h"

/// Struct defined in msg/RoadFeature2D in the package kaiev26_msgs.
typedef struct kaiev26_msgs__msg__RoadFeature2D
{
  /// frame-local ID, road_features 배열 안에서 중복 금지
  uint32_t id;
  /// KIND_* feature 종류
  uint8_t kind;
  /// COLOR_* 표시 색상
  uint8_t color;
  /// STYLE_* 선 형태, 횡단보도 등 미적용시 STYLE_UNKNOWN
  uint8_t style;
  /// feature confidence [0, 1]
  float score;
  /// lane/stop line은 false, crosswalk polygon은 true
  bool closed;
  /// 원본 image pixel; x=u 우측, y=v 하측, z=0 고정
  geometry_msgs__msg__Point32__Sequence points;
} kaiev26_msgs__msg__RoadFeature2D;

// Struct for a sequence of kaiev26_msgs__msg__RoadFeature2D.
typedef struct kaiev26_msgs__msg__RoadFeature2D__Sequence
{
  kaiev26_msgs__msg__RoadFeature2D * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__RoadFeature2D__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__ROAD_FEATURE2_D__STRUCT_H_
