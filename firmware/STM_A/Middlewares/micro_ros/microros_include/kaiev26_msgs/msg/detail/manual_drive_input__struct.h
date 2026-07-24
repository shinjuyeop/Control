// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/ManualDriveInput.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__MANUAL_DRIVE_INPUT__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__MANUAL_DRIVE_INPUT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'GEAR_FORWARD'.
enum
{
  kaiev26_msgs__msg__ManualDriveInput__GEAR_FORWARD = 0
};

/// Constant 'GEAR_REVERSE'.
enum
{
  kaiev26_msgs__msg__ManualDriveInput__GEAR_REVERSE = 1
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/ManualDriveInput in the package kaiev26_msgs.
/**
  * 수동주행(사람 입력) 모사용 sim 전용 명령 (/simulation/manual_input).
  * 실차에서는 가속페달·기어 스위치가 ROS를 거치지 않고 STM_A로 직접 가서
  * 모터 토크(전류) 명령이 되므로, 이 메시지는 공통 제어 계약이 아니다.
  * 입력이 끊기면 플랜트는 AUTO 경로(/vehicle/actuator_command)로 복귀한다.
 */
typedef struct kaiev26_msgs__msg__ManualDriveInput
{
  std_msgs__msg__Header header;
  float steering_target_rad;
  /// 가속페달 비율 0..1. 실차 /pedal 0-100%에 대응하며 토크 한계에 곱해진다.
  float pedal_ratio;
  bool brake_engage;
  uint8_t gear;
} kaiev26_msgs__msg__ManualDriveInput;

// Struct for a sequence of kaiev26_msgs__msg__ManualDriveInput.
typedef struct kaiev26_msgs__msg__ManualDriveInput__Sequence
{
  kaiev26_msgs__msg__ManualDriveInput * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__ManualDriveInput__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__MANUAL_DRIVE_INPUT__STRUCT_H_
