// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from throttle_msgs:msg/ThrottleData.idl
// generated code does not contain a copyright notice

#ifndef THROTTLE_MSGS__MSG__DETAIL__THROTTLE_DATA__STRUCT_H_
#define THROTTLE_MSGS__MSG__DETAIL__THROTTLE_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ThrottleData in the package throttle_msgs.
typedef struct throttle_msgs__msg__ThrottleData
{
  int16_t rpm;
  bool estop;
  bool asms;
} throttle_msgs__msg__ThrottleData;

// Struct for a sequence of throttle_msgs__msg__ThrottleData.
typedef struct throttle_msgs__msg__ThrottleData__Sequence
{
  throttle_msgs__msg__ThrottleData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} throttle_msgs__msg__ThrottleData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // THROTTLE_MSGS__MSG__DETAIL__THROTTLE_DATA__STRUCT_H_
