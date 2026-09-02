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
  /// MODE_MANUAL / MODE_AUTO. 물리 스위치
  uint8_t mode;
  /// GEAR_FORWARD / GEAR_REVERSE. 물리 스위치
  uint8_t gear;
  /// 운전자 가속페달 0~100%. MANUAL 상전류 지령의 원천
  uint8_t pedal_percent;
  /// 데드밴드 클램핑 전 ADC 원시값. 재보정 후 로그 재계산용
  int32_t pedal_adc_raw;
  /// 모터 RPM 기반 추정 차량 속도
  float speed_mps;
  float motor_rpm;
  bool tractive_commanded;
  /// 드라이버 RUN 상태 (Part II byte 2 bit0)
  bool tractive_feedback;
  /// Part II 미수신 구간에서 false
  bool tractive_feedback_valid;
  bool stop_request;
  /// EZkontrol Part II가 보내는 24비트 fault 원본
  uint32_t fault_bits;
  /// Part II 미수신 구간에서 false
  bool fault_bits_valid;
  /// Host 명령 수신 상태
  bool command_received;
  bool command_timeout;
  float command_age_ms;
  /// STM_A가 EZkontrol로 내보낸 명령 (0x0C01EFD0 byte 0-1, byte 2-3).
  /// 인코딩과 클램핑을 거친 뒤의 값이라 버스에 실제로 실린 값과 같다.
  /// MANUAL(torque)에서 목표, AUTO(speed)에서 상한
  float commanded_phase_current_a;
  /// AUTO(speed)에서 목표, MANUAL(torque)에서 상한
  float commanded_motor_rpm;
  /// EZkontrol status Part I (0x1801D0EF)
  float bus_voltage_v;
  float bus_current_a;
  /// 드라이버가 실제로 흘린 상전류. 토크에 비례
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
