// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/DriveIntent.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__DRIVE_INTENT__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__DRIVE_INTENT__STRUCT_H_

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

/// Struct defined in msg/DriveIntent in the package kaiev26_msgs.
typedef struct kaiev26_msgs__msg__DriveIntent
{
  std_msgs__msg__Header header;
  float target_speed_mps;
  float target_steering_rad;
  bool brake_request;
  bool enable;
} kaiev26_msgs__msg__DriveIntent;

// Struct for a sequence of kaiev26_msgs__msg__DriveIntent.
typedef struct kaiev26_msgs__msg__DriveIntent__Sequence
{
  kaiev26_msgs__msg__DriveIntent * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__DriveIntent__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__DRIVE_INTENT__STRUCT_H_
