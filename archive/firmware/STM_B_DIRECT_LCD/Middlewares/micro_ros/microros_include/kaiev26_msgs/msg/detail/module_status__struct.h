// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/ModuleStatus.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__MODULE_STATUS__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__MODULE_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MODULE_UNKNOWN'.
/**
  * 미지정, ready payload에서는 사용 금지
 */
enum
{
  kaiev26_msgs__msg__ModuleStatus__MODULE_UNKNOWN = 0
};

/// Constant 'MODULE_SEGMENTATION'.
/**
  * drivable/lane segmentation
 */
enum
{
  kaiev26_msgs__msg__ModuleStatus__MODULE_SEGMENTATION = 1
};

/// Constant 'MODULE_DETECTOR'.
/**
  * 7-class object detector
 */
enum
{
  kaiev26_msgs__msg__ModuleStatus__MODULE_DETECTOR = 2
};

/// Constant 'MODULE_ROADMARK'.
/**
  * lane/stop-line/crosswalk geometry 생성
 */
enum
{
  kaiev26_msgs__msg__ModuleStatus__MODULE_ROADMARK = 3
};

/// Constant 'MODULE_TRAFFIC_LIGHT_ATTR'.
/**
  * 신호등 색상 및 화살표 attribute
 */
enum
{
  kaiev26_msgs__msg__ModuleStatus__MODULE_TRAFFIC_LIGHT_ATTR = 4
};

/// Constant 'STATE_UNKNOWN'.
/**
  * 상태 미지정, ready payload에서는 사용 금지
 */
enum
{
  kaiev26_msgs__msg__ModuleStatus__STATE_UNKNOWN = 0
};

/// Constant 'STATE_OK'.
/**
  * 정상 실행되어 유효 결과가 하나 이상 있음
 */
enum
{
  kaiev26_msgs__msg__ModuleStatus__STATE_OK = 1
};

/// Constant 'STATE_EMPTY'.
/**
  * 정상 실행되었으나 유효 결과가 0개임
 */
enum
{
  kaiev26_msgs__msg__ModuleStatus__STATE_EMPTY = 2
};

/// Constant 'STATE_TIMEOUT'.
/**
  * frame deadline 안에 결과를 완성하지 못함
 */
enum
{
  kaiev26_msgs__msg__ModuleStatus__STATE_TIMEOUT = 3
};

/// Constant 'STATE_ERROR'.
/**
  * 실행 예외, malformed/nonfinite 출력 또는 계약 검증 실패
 */
enum
{
  kaiev26_msgs__msg__ModuleStatus__STATE_ERROR = 4
};

/// Constant 'STATE_DISABLED'.
/**
  * 의도적 비활성, 해당 frontend가 선언한 required module에는 금지
 */
enum
{
  kaiev26_msgs__msg__ModuleStatus__STATE_DISABLED = 5
};

// Include directives for member types
// Member 'model_id'
// Member 'detail'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/ModuleStatus in the package kaiev26_msgs.
typedef struct kaiev26_msgs__msg__ModuleStatus
{
  /// MODULE_* 모듈 식별자, 같은 frame에서 중복 금지
  uint8_t module;
  /// STATE_* 실행 결과, consumer 분기는 이 값만 사용
  uint8_t state;
  /// callback 수신부터 모듈 완료까지 지연, 미측정시 NaN
  float latency_ms;
  /// manifest version과 weight SHA256 prefix를 포함한 model 식별자
  rosidl_runtime_c__String model_id;
  /// 사람용 진단 문자열, consumer 제어 분기에 사용 금지
  rosidl_runtime_c__String detail;
} kaiev26_msgs__msg__ModuleStatus;

// Struct for a sequence of kaiev26_msgs__msg__ModuleStatus.
typedef struct kaiev26_msgs__msg__ModuleStatus__Sequence
{
  kaiev26_msgs__msg__ModuleStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__ModuleStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__MODULE_STATUS__STRUCT_H_
