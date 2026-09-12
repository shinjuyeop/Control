// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/RouteContext.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__ROUTE_CONTEXT__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__ROUTE_CONTEXT__STRUCT_H_

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
// Member 'current_zone'
// Member 'next_zone'
// Member 'next_maneuver'
// Member 'current_lanelet_id'
// Member 'current_road_code'
// Member 'current_intersection_id'
// Member 'next_stop_line_id'
// Member 'next_traffic_light_ids'
#include "rosidl_runtime_c/string.h"
// Member 'next_traffic_light_positions'
// Member 'local_path_points'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/RouteContext in the package kaiev26_msgs.
typedef struct kaiev26_msgs__msg__RouteContext
{
  std_msgs__msg__Header header;
  float progress_s;
  rosidl_runtime_c__String current_zone;
  rosidl_runtime_c__String next_zone;
  rosidl_runtime_c__String next_maneuver;
  rosidl_runtime_c__String current_lanelet_id;
  rosidl_runtime_c__String current_road_code;
  rosidl_runtime_c__String current_intersection_id;
  rosidl_runtime_c__String next_stop_line_id;
  rosidl_runtime_c__String__Sequence next_traffic_light_ids;
  geometry_msgs__msg__Point__Sequence next_traffic_light_positions;
  float distance_to_stopline;
  float distance_to_intersection;
  float distance_to_finish;
  float speed_limit_mps;
  bool route_projection_valid;
  float projection_confidence;
  float cross_track_error;
  float heading_error;
  geometry_msgs__msg__Point__Sequence local_path_points;
} kaiev26_msgs__msg__RouteContext;

// Struct for a sequence of kaiev26_msgs__msg__RouteContext.
typedef struct kaiev26_msgs__msg__RouteContext__Sequence
{
  kaiev26_msgs__msg__RouteContext * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__RouteContext__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__ROUTE_CONTEXT__STRUCT_H_
