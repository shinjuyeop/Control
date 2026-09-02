// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/PerceptionObject.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__PERCEPTION_OBJECT__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__PERCEPTION_OBJECT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'CLASS_VEHICLE'.
enum
{
  kaiev26_msgs__msg__PerceptionObject__CLASS_VEHICLE = 1
};

/// Constant 'CLASS_BIKE'.
enum
{
  kaiev26_msgs__msg__PerceptionObject__CLASS_BIKE = 2
};

/// Constant 'CLASS_PEDESTRIAN'.
enum
{
  kaiev26_msgs__msg__PerceptionObject__CLASS_PEDESTRIAN = 3
};

/// Constant 'CLASS_TRAFFIC_CONE'.
enum
{
  kaiev26_msgs__msg__PerceptionObject__CLASS_TRAFFIC_CONE = 4
};

/// Constant 'CLASS_OBSTACLE'.
enum
{
  kaiev26_msgs__msg__PerceptionObject__CLASS_OBSTACLE = 5
};

/// Constant 'CLASS_TRAFFIC_LIGHT'.
enum
{
  kaiev26_msgs__msg__PerceptionObject__CLASS_TRAFFIC_LIGHT = 6
};

/// Constant 'CLASS_SIGN'.
enum
{
  kaiev26_msgs__msg__PerceptionObject__CLASS_SIGN = 7
};

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'size'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'twist'
#include "geometry_msgs/msg/detail/twist__struct.h"
// Member 'source'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/PerceptionObject in the package kaiev26_msgs.
typedef struct kaiev26_msgs__msg__PerceptionObject
{
  uint32_t detection_id;
  uint32_t track_id;
  uint8_t class_id;
  geometry_msgs__msg__Pose pose;
  geometry_msgs__msg__Vector3 size;
  geometry_msgs__msg__Twist twist;
  float confidence;
  rosidl_runtime_c__String source;
} kaiev26_msgs__msg__PerceptionObject;

// Struct for a sequence of kaiev26_msgs__msg__PerceptionObject.
typedef struct kaiev26_msgs__msg__PerceptionObject__Sequence
{
  kaiev26_msgs__msg__PerceptionObject * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__PerceptionObject__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__PERCEPTION_OBJECT__STRUCT_H_
