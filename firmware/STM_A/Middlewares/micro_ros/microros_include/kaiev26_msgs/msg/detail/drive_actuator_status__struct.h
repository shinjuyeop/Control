// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/DriveActuatorStatus.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__DRIVE_ACTUATOR_STATUS__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__DRIVE_ACTUATOR_STATUS__STRUCT_H_

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
  kaiev26_msgs__msg__DriveActuatorStatus__GEAR_FORWARD = 0
};

/// Constant 'GEAR_REVERSE'.
enum
{
  kaiev26_msgs__msg__DriveActuatorStatus__GEAR_REVERSE = 1
};

/// Constant 'MODE_MANUAL'.
enum
{
  kaiev26_msgs__msg__DriveActuatorStatus__MODE_MANUAL = 0
};

/// Constant 'MODE_AUTO'.
enum
{
  kaiev26_msgs__msg__DriveActuatorStatus__MODE_AUTO = 1
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/DriveActuatorStatus in the package kaiev26_msgs.
/**
  * 구동(STM_A + 모터드라이버) 진단 status (D-10·D-11).
  * 실차 STM_A가 현재 제공 가능한 값만 유효로 채우고, 미확인 값은 *_valid=false.
 */
typedef struct kaiev26_msgs__msg__DriveActuatorStatus
{
  std_msgs__msg__Header header;
  uint8_t mode;
  uint8_t gear;
  /// 모터 RPM 기반 추정 차량 속도
  float speed_mps;
  float motor_rpm;
  bool tractive_commanded;
  /// 실차는 드라이버 RUN feedback 지원 확인 전까지 valid=false
  bool tractive_feedback;
  bool tractive_feedback_valid;
  bool stop_request;
  /// EZkontrol Part II가 보내는 24비트 fault 원본
  uint32_t fault_bits;
  bool fault_bits_valid;
  /// Host 명령 수신 상태
  bool command_received;
  bool command_timeout;
  float command_age_ms;
  /// EZkontrol status Part I (0x1801D0EF)
  float bus_voltage_v;
  float bus_current_a;
  float phase_current_a;
  bool part1_valid;
  float part1_age_ms;
  /// EZkontrol status Part II (0x1802D0EF)
  float controller_temperature_c;
  float motor_temperature_c;
  /// 0=torque control, 1=speed control
  uint8_t controller_control_mode;
  bool part2_valid;
  float part2_age_ms;
} kaiev26_msgs__msg__DriveActuatorStatus;

// Struct for a sequence of kaiev26_msgs__msg__DriveActuatorStatus.
typedef struct kaiev26_msgs__msg__DriveActuatorStatus__Sequence
{
  kaiev26_msgs__msg__DriveActuatorStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__DriveActuatorStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__DRIVE_ACTUATOR_STATUS__STRUCT_H_
