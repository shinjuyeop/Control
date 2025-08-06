// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from throttle_msgs:msg/ThrottleData.idl
// generated code does not contain a copyright notice
#include "throttle_msgs/msg/detail/throttle_data__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "throttle_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "throttle_msgs/msg/detail/throttle_data__struct.h"
#include "throttle_msgs/msg/detail/throttle_data__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _ThrottleData__ros_msg_type = throttle_msgs__msg__ThrottleData;

static bool _ThrottleData__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ThrottleData__ros_msg_type * ros_message = static_cast<const _ThrottleData__ros_msg_type *>(untyped_ros_message);
  // Field name: rpm
  {
    cdr << ros_message->rpm;
  }

  // Field name: estop
  {
    cdr << (ros_message->estop ? true : false);
  }

  // Field name: asms
  {
    cdr << (ros_message->asms ? true : false);
  }

  return true;
}

static bool _ThrottleData__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ThrottleData__ros_msg_type * ros_message = static_cast<_ThrottleData__ros_msg_type *>(untyped_ros_message);
  // Field name: rpm
  {
    cdr >> ros_message->rpm;
  }

  // Field name: estop
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->estop = tmp ? true : false;
  }

  // Field name: asms
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->asms = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_throttle_msgs
size_t get_serialized_size_throttle_msgs__msg__ThrottleData(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ThrottleData__ros_msg_type * ros_message = static_cast<const _ThrottleData__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name rpm
  {
    size_t item_size = sizeof(ros_message->rpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name estop
  {
    size_t item_size = sizeof(ros_message->estop);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name asms
  {
    size_t item_size = sizeof(ros_message->asms);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ThrottleData__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_throttle_msgs__msg__ThrottleData(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_throttle_msgs
size_t max_serialized_size_throttle_msgs__msg__ThrottleData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: rpm
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: estop
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: asms
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = throttle_msgs__msg__ThrottleData;
    is_plain =
      (
      offsetof(DataType, asms) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _ThrottleData__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_throttle_msgs__msg__ThrottleData(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ThrottleData = {
  "throttle_msgs::msg",
  "ThrottleData",
  _ThrottleData__cdr_serialize,
  _ThrottleData__cdr_deserialize,
  _ThrottleData__get_serialized_size,
  _ThrottleData__max_serialized_size
};

static rosidl_message_type_support_t _ThrottleData__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ThrottleData,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, throttle_msgs, msg, ThrottleData)() {
  return &_ThrottleData__type_support;
}

#if defined(__cplusplus)
}
#endif
