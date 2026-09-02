// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/Centerline.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__CENTERLINE__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__CENTERLINE__STRUCT_H_

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
// Member 'points'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'source'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Centerline in the package kaiev26_msgs.
typedef struct kaiev26_msgs__msg__Centerline
{
  std_msgs__msg__Header header;
  uint32_t detection_id;
  uint32_t track_id;
  geometry_msgs__msg__Point__Sequence points;
  float confidence;
  rosidl_runtime_c__String source;
} kaiev26_msgs__msg__Centerline;

// Struct for a sequence of kaiev26_msgs__msg__Centerline.
typedef struct kaiev26_msgs__msg__Centerline__Sequence
{
  kaiev26_msgs__msg__Centerline * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__Centerline__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__CENTERLINE__STRUCT_H_
