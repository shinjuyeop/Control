// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/SceneSummary.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__SCENE_SUMMARY__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__SCENE_SUMMARY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'TRAFFIC_UNKNOWN'.
enum
{
  kaiev26_msgs__msg__SceneSummary__TRAFFIC_UNKNOWN = 0
};

/// Constant 'TRAFFIC_RED'.
enum
{
  kaiev26_msgs__msg__SceneSummary__TRAFFIC_RED = 1
};

/// Constant 'TRAFFIC_YELLOW'.
enum
{
  kaiev26_msgs__msg__SceneSummary__TRAFFIC_YELLOW = 2
};

/// Constant 'TRAFFIC_GREEN'.
enum
{
  kaiev26_msgs__msg__SceneSummary__TRAFFIC_GREEN = 3
};

/// Constant 'TRAFFIC_ARROW'.
enum
{
  kaiev26_msgs__msg__SceneSummary__TRAFFIC_ARROW = 4
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'centerline_points'
// Member 'left_lane_center_points'
// Member 'right_lane_center_points'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'obstacle_risk_level'
// Member 'perception_health'
// Member 'events'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/SceneSummary in the package kaiev26_msgs.
typedef struct kaiev26_msgs__msg__SceneSummary
{
  std_msgs__msg__Header header;
  bool lane_valid;
  bool centerline_valid;
  float centerline_quality;
  float centerline_visible_length;
  geometry_msgs__msg__Point__Sequence centerline_points;
  bool left_lane_center_valid;
  geometry_msgs__msg__Point__Sequence left_lane_center_points;
  bool right_lane_center_valid;
  geometry_msgs__msg__Point__Sequence right_lane_center_points;
  float lane_width_estimate_m;
  float stopline_distance;
  uint8_t traffic_light_state;
  float traffic_light_confidence;
  bool obstacle_on_path;
  float front_obstacle_distance;
  float front_obstacle_x;
  float front_obstacle_y;
  float front_obstacle_length;
  float front_obstacle_width;
  rosidl_runtime_c__String obstacle_risk_level;
  rosidl_runtime_c__String perception_health;
  float oldest_data_age_ms;
  rosidl_runtime_c__String__Sequence events;
} kaiev26_msgs__msg__SceneSummary;

// Struct for a sequence of kaiev26_msgs__msg__SceneSummary.
typedef struct kaiev26_msgs__msg__SceneSummary__Sequence
{
  kaiev26_msgs__msg__SceneSummary * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__SceneSummary__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__SCENE_SUMMARY__STRUCT_H_
