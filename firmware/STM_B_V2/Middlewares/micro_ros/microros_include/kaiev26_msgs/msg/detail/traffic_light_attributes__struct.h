// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from kaiev26_msgs:msg/TrafficLightAttributes.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__TRAFFIC_LIGHT_ATTRIBUTES__STRUCT_H_
#define KAIEV26_MSGS__MSG__DETAIL__TRAFFIC_LIGHT_ATTRIBUTES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'INVALID_NONE'.
/**
  * valid=true인 정상 attribute
 */
enum
{
  kaiev26_msgs__msg__TrafficLightAttributes__INVALID_NONE = 0
};

/// Constant 'INVALID_NOT_APPLICABLE'.
/**
  * 신호등 class가 아니어서 attribute 미적용
 */
enum
{
  kaiev26_msgs__msg__TrafficLightAttributes__INVALID_NOT_APPLICABLE = 1
};

/// Constant 'INVALID_ROI'.
/**
  * 신호등 ROI가 비어 있거나 image 경계 밖이라 추론 불가
 */
enum
{
  kaiev26_msgs__msg__TrafficLightAttributes__INVALID_ROI = 2
};

/// Constant 'INVALID_LOW_CONFIDENCE'.
/**
  * attribute 신뢰도가 사용 기준 미만
 */
enum
{
  kaiev26_msgs__msg__TrafficLightAttributes__INVALID_LOW_CONFIDENCE = 3
};

/// Constant 'INVALID_AMBIGUOUS'.
/**
  * 상충하는 색상 score로 단일 상태를 확정할 수 없음
 */
enum
{
  kaiev26_msgs__msg__TrafficLightAttributes__INVALID_AMBIGUOUS = 4
};

/// Constant 'INVALID_MODEL_ERROR'.
/**
  * attribute model 실행 또는 출력 검증 실패
 */
enum
{
  kaiev26_msgs__msg__TrafficLightAttributes__INVALID_MODEL_ERROR = 5
};

/// Struct defined in msg/TrafficLightAttributes in the package kaiev26_msgs.
typedef struct kaiev26_msgs__msg__TrafficLightAttributes
{
  /// 네 score를 관측값으로 사용할 수 있으면 true
  bool valid;
  /// 적색 marginal score [0, 1], valid=false이면 미사용시 NaN
  float red_score;
  /// 황색 marginal score [0, 1], valid=false이면 미사용시 NaN
  float yellow_score;
  /// 녹색 marginal score [0, 1], valid=false이면 미사용시 NaN
  float green_score;
  /// 화살표 독립 score [0, 1], valid=false이면 미사용시 NaN
  float arrow_score;
  /// valid=false인 이유, valid=true이면 INVALID_NONE
  uint8_t invalid_reason;
} kaiev26_msgs__msg__TrafficLightAttributes;

// Struct for a sequence of kaiev26_msgs__msg__TrafficLightAttributes.
typedef struct kaiev26_msgs__msg__TrafficLightAttributes__Sequence
{
  kaiev26_msgs__msg__TrafficLightAttributes * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} kaiev26_msgs__msg__TrafficLightAttributes__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__TRAFFIC_LIGHT_ATTRIBUTES__STRUCT_H_
