// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/Perception2DFrame.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__PERCEPTION2_D_FRAME__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__PERCEPTION2_D_FRAME__STRUCT_H_

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
// Member 'detections'
#include "kaiev26_msgs/msg/detail/detection2_d__struct.h"
// Member 'road_features'
#include "kaiev26_msgs/msg/detail/road_feature2_d__struct.h"
// Member 'module_statuses'
#include "kaiev26_msgs/msg/detail/module_status__struct.h"

/// Struct defined in msg/Perception2DFrame in the package kaiev26_msgs.
typedef struct kaiev26_msgs__msg__Perception2DFrame
{
  /// source image stamp와 optical frame_id를 그대로 보존
  std_msgs__msg__Header header;
  /// 원본 image 너비, 1 이상
  uint32_t image_width;
  /// 원본 image 높이, 1 이상
  uint32_t image_height;
  /// CameraInfo.P와 정합된 rectified image이면 true, production은 true 필수
  bool rectified;
  /// node 수신 순서, 0부터 증가하며 drop 이후에도 재사용 금지
  uint64_t input_sequence;
  /// callback 수신부터 이 frame publish 완료까지, 0 이상
  float processing_ms;
  /// processing_ms가 설정된 frame deadline을 초과하면 true
  bool deadline_missed;
  /// required module 중 STATE_TIMEOUT 또는 STATE_ERROR가 하나라도 있으면 true
  bool partial;
  /// 이 source frame의 2D object 관측, stale 결과 혼합 금지
  kaiev26_msgs__msg__Detection2D__Sequence detections;
  /// 이 source frame의 차선/정지선/횡단보도 image geometry
  kaiev26_msgs__msg__RoadFeature2D__Sequence road_features;
  /// frontend 선언 module별 1개 상태, optional은 DISABLED로 명시
  kaiev26_msgs__msg__ModuleStatus__Sequence module_statuses;
} kaiev26_msgs__msg__Perception2DFrame;

// Struct for a sequence of kaiev26_msgs__msg__Perception2DFrame.
typedef struct kaiev26_msgs__msg__Perception2DFrame__Sequence
{
  kaiev26_msgs__msg__Perception2DFrame * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__Perception2DFrame__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__PERCEPTION2_D_FRAME__STRUCT_H_
