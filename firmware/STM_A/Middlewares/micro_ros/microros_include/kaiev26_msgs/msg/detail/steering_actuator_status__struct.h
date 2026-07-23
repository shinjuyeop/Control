// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/SteeringActuatorStatus.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__STEERING_ACTUATOR_STATUS__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__STEERING_ACTUATOR_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STATE_INIT'.
/**
  * state enum (실차 steering_actuator_node와 동일)
 */
enum
{
  kaiev26_msgs__msg__SteeringActuatorStatus__STATE_INIT = 0
};

/// Constant 'STATE_DISABLED'.
enum
{
  kaiev26_msgs__msg__SteeringActuatorStatus__STATE_DISABLED = 1
};

/// Constant 'STATE_WAIT_CMD'.
enum
{
  kaiev26_msgs__msg__SteeringActuatorStatus__STATE_WAIT_CMD = 2
};

/// Constant 'STATE_ACTIVE'.
enum
{
  kaiev26_msgs__msg__SteeringActuatorStatus__STATE_ACTIVE = 3
};

/// Constant 'STATE_ESTOP'.
enum
{
  kaiev26_msgs__msg__SteeringActuatorStatus__STATE_ESTOP = 4
};

/// Constant 'STATE_AEB'.
enum
{
  kaiev26_msgs__msg__SteeringActuatorStatus__STATE_AEB = 5
};

/// Constant 'STATE_CAN_LOST'.
enum
{
  kaiev26_msgs__msg__SteeringActuatorStatus__STATE_CAN_LOST = 6
};

/// Constant 'STATE_FEEDBACK_LOST'.
enum
{
  kaiev26_msgs__msg__SteeringActuatorStatus__STATE_FEEDBACK_LOST = 7
};

/// Constant 'STATE_FAULT'.
enum
{
  kaiev26_msgs__msg__SteeringActuatorStatus__STATE_FAULT = 8
};

/// Constant 'REASON_NONE'.
/**
  * disable_reason enum (실차와 동일)
 */
enum
{
  kaiev26_msgs__msg__SteeringActuatorStatus__REASON_NONE = 0
};

/// Constant 'REASON_MODE_NOT_RECEIVED'.
enum
{
  kaiev26_msgs__msg__SteeringActuatorStatus__REASON_MODE_NOT_RECEIVED = 1
};

/// Constant 'REASON_MODE_TIMEOUT'.
enum
{
  kaiev26_msgs__msg__SteeringActuatorStatus__REASON_MODE_TIMEOUT = 2
};

/// Constant 'REASON_MANUAL_MODE'.
enum
{
  kaiev26_msgs__msg__SteeringActuatorStatus__REASON_MANUAL_MODE = 3
};

/// Constant 'REASON_INVALID_MODE'.
enum
{
  kaiev26_msgs__msg__SteeringActuatorStatus__REASON_INVALID_MODE = 4
};

/// Constant 'REASON_ESTOP'.
enum
{
  kaiev26_msgs__msg__SteeringActuatorStatus__REASON_ESTOP = 5
};

/// Constant 'REASON_AEB'.
enum
{
  kaiev26_msgs__msg__SteeringActuatorStatus__REASON_AEB = 6
};

/// Constant 'REASON_STEER_COMMAND_TIMEOUT'.
enum
{
  kaiev26_msgs__msg__SteeringActuatorStatus__REASON_STEER_COMMAND_TIMEOUT = 7
};

/// Constant 'REASON_CAN_TIMEOUT'.
enum
{
  kaiev26_msgs__msg__SteeringActuatorStatus__REASON_CAN_TIMEOUT = 8
};

/// Constant 'REASON_FEEDBACK_INVALID'.
enum
{
  kaiev26_msgs__msg__SteeringActuatorStatus__REASON_FEEDBACK_INVALID = 9
};

/// Constant 'REASON_MOTOR_FAULT'.
enum
{
  kaiev26_msgs__msg__SteeringActuatorStatus__REASON_MOTOR_FAULT = 10
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/SteeringActuatorStatus in the package kaiev26_msgs.
/**
  * 조향 액추에이터 진단 status (D-10·D-11). 실차 lateral_ctrl의
  * SteeringActuatorStatus(533a3fb9) 설계를 계승하되 canonical 단위로 정리:
  * 각도는 rad·좌회전 양수, 각속도는 rad/s. deg·부호 반전은 실차 어댑터 내부에서만.
  * 실차 통합 시 lateral_ctrl 로컬 메시지를 삭제하고 이 메시지로 교체한다(D-15 SSOT).
 */
typedef struct kaiev26_msgs__msg__SteeringActuatorStatus
{
  std_msgs__msg__Header header;
  uint8_t state;
  uint8_t disable_reason;
  uint8_t drive_mode;
  bool mode_received;
  bool mode_timeout;
  bool steer_command_timeout;
  bool feedback_timeout;
  bool steer_command_received;
  bool pending_command_valid;
  bool movement_target_valid;
  bool estop_active;
  bool aeb_active;
  bool communication_ok;
  bool can_timeout;
  bool feedback_valid;
  bool enabled_commanded;
  bool enabled_feedback;
  /// 실차 KEYA는 신뢰 가능한 enable query가 없어 false. sim은 참값 제공 가능
  bool enabled_feedback_valid;
  bool enable_mismatch;
  bool tracking_control_enabled;
  bool tracking_error_warning;
  bool tracking_error_fault;
  float target_wheel_angle_rad;
  float commanded_wheel_angle_rad;
  /// tracking PID 보정이 적용된 최종 명령각
  float corrected_wheel_angle_rad;
  float actual_wheel_angle_rad;
  float tracking_error_rad;
  float tracking_correction_rad;
  float steering_rate_radps;
  int32_t motor_position_count;
  /// 모터축 각도(원시 진단값, count 파생). 로드휠 부호 변환을 적용하지 않는다
  float motor_angle_deg;
  float motor_speed_rpm;
  float motor_current_a;
  float bus_voltage_v;
  float driver_temperature_c;
  uint16_t fault_bits;
  float feedback_age_ms;
  float mode_age_ms;
  float steer_command_age_ms;
} kaiev26_msgs__msg__SteeringActuatorStatus;

// Struct for a sequence of kaiev26_msgs__msg__SteeringActuatorStatus.
typedef struct kaiev26_msgs__msg__SteeringActuatorStatus__Sequence
{
  kaiev26_msgs__msg__SteeringActuatorStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__SteeringActuatorStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__STEERING_ACTUATOR_STATUS__STRUCT_H_
