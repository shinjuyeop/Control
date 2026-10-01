// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/DriveCurrentCommand.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__DRIVE_CURRENT_COMMAND__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__DRIVE_CURRENT_COMMAND__STRUCT_H_

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

/// Struct defined in msg/DriveCurrentCommand in the package kaiev26_msgs.
/**
  * Forward AUTO current demand. Consumed only in external_current mode.
  * Header uses the publisher's ROS clock; phase_current_a is motor phase amperes.
 */
typedef struct kaiev26_msgs__msg__DriveCurrentCommand
{
  std_msgs__msg__Header header;
  float phase_current_a;
} kaiev26_msgs__msg__DriveCurrentCommand;

// Struct for a sequence of kaiev26_msgs__msg__DriveCurrentCommand.
typedef struct kaiev26_msgs__msg__DriveCurrentCommand__Sequence
{
  kaiev26_msgs__msg__DriveCurrentCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__DriveCurrentCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__DRIVE_CURRENT_COMMAND__STRUCT_H_
