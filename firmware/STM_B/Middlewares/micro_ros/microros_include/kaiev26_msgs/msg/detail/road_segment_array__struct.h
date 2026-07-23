// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/RoadSegmentArray.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__ROAD_SEGMENT_ARRAY__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__ROAD_SEGMENT_ARRAY__STRUCT_H_

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
// Member 'segments'
#include "kaiev26_msgs/msg/detail/road_segment__struct.h"

/// Struct defined in msg/RoadSegmentArray in the package kaiev26_msgs.
typedef struct kaiev26_msgs__msg__RoadSegmentArray
{
  std_msgs__msg__Header header;
  kaiev26_msgs__msg__RoadSegment__Sequence segments;
} kaiev26_msgs__msg__RoadSegmentArray;

// Struct for a sequence of kaiev26_msgs__msg__RoadSegmentArray.
typedef struct kaiev26_msgs__msg__RoadSegmentArray__Sequence
{
  kaiev26_msgs__msg__RoadSegmentArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__RoadSegmentArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__ROAD_SEGMENT_ARRAY__STRUCT_H_
