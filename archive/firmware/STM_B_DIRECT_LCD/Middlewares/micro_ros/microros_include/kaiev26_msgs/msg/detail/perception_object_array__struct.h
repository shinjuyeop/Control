// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/PerceptionObjectArray.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__PERCEPTION_OBJECT_ARRAY__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__PERCEPTION_OBJECT_ARRAY__STRUCT_H_

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
// Member 'objects'
#include "kaiev26_msgs/msg/detail/perception_object__struct.h"

/// Struct defined in msg/PerceptionObjectArray in the package kaiev26_msgs.
typedef struct kaiev26_msgs__msg__PerceptionObjectArray
{
  std_msgs__msg__Header header;
  kaiev26_msgs__msg__PerceptionObject__Sequence objects;
} kaiev26_msgs__msg__PerceptionObjectArray;

// Struct for a sequence of kaiev26_msgs__msg__PerceptionObjectArray.
typedef struct kaiev26_msgs__msg__PerceptionObjectArray__Sequence
{
  kaiev26_msgs__msg__PerceptionObjectArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__PerceptionObjectArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__PERCEPTION_OBJECT_ARRAY__STRUCT_H_
