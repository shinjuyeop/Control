// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/Detection2D.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__DETECTION2_D__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__DETECTION2_D__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'CLASS_VEHICLE'.
/**
  * 자동차
 */
enum
{
  kaiev26_msgs__msg__Detection2D__CLASS_VEHICLE = 0
};

/// Constant 'CLASS_BIKE'.
/**
  * 자전거 또는 이륜차
 */
enum
{
  kaiev26_msgs__msg__Detection2D__CLASS_BIKE = 1
};

/// Constant 'CLASS_PEDESTRIAN'.
/**
  * 보행자
 */
enum
{
  kaiev26_msgs__msg__Detection2D__CLASS_PEDESTRIAN = 2
};

/// Constant 'CLASS_TRAFFIC_CONE'.
/**
  * 러버콘
 */
enum
{
  kaiev26_msgs__msg__Detection2D__CLASS_TRAFFIC_CONE = 3
};

/// Constant 'CLASS_OBSTACLE'.
/**
  * 그 밖의 정적 또는 동적 장애물
 */
enum
{
  kaiev26_msgs__msg__Detection2D__CLASS_OBSTACLE = 4
};

/// Constant 'CLASS_TRAFFIC_LIGHT'.
/**
  * 신호등
 */
enum
{
  kaiev26_msgs__msg__Detection2D__CLASS_TRAFFIC_LIGHT = 5
};

/// Constant 'CLASS_SIGN'.
/**
  * 교통 표지판
 */
enum
{
  kaiev26_msgs__msg__Detection2D__CLASS_SIGN = 6
};

// Include directives for member types
// Member 'traffic_light'
#include "kaiev26_msgs/msg/detail/traffic_light_attributes__struct.h"

/// Struct defined in msg/Detection2D in the package kaiev26_msgs.
typedef struct kaiev26_msgs__msg__Detection2D
{
  /// frame-local ID, 같은 frame에서 0부터 연속이며 중복 금지
  uint32_t id;
  /// 원본 image pixel [x1, y1, x2, y2], 좌상단 원점, x1<x2 및 y1<y2
  float box_xyxy[4];
  /// detection confidence [0, 1]
  float score;
  /// canonical 0..6 class ID
  uint8_t class_id;
  /// 신호등 속성, 비신호등은 valid=false/INVALID_NOT_APPLICABLE
  kaiev26_msgs__msg__TrafficLightAttributes traffic_light;
} kaiev26_msgs__msg__Detection2D;

// Struct for a sequence of kaiev26_msgs__msg__Detection2D.
typedef struct kaiev26_msgs__msg__Detection2D__Sequence
{
  kaiev26_msgs__msg__Detection2D * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__Detection2D__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__DETECTION2_D__STRUCT_H_
