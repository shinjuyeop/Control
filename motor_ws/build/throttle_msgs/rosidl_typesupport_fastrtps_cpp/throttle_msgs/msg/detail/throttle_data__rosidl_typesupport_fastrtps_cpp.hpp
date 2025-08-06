// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from throttle_msgs:msg/ThrottleData.idl
// generated code does not contain a copyright notice

#ifndef THROTTLE_MSGS__MSG__DETAIL__THROTTLE_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define THROTTLE_MSGS__MSG__DETAIL__THROTTLE_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "throttle_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "throttle_msgs/msg/detail/throttle_data__struct.hpp"

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

#include "fastcdr/Cdr.h"

namespace throttle_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_throttle_msgs
cdr_serialize(
  const throttle_msgs::msg::ThrottleData & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_throttle_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  throttle_msgs::msg::ThrottleData & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_throttle_msgs
get_serialized_size(
  const throttle_msgs::msg::ThrottleData & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_throttle_msgs
max_serialized_size_ThrottleData(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace throttle_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_throttle_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, throttle_msgs, msg, ThrottleData)();

#ifdef __cplusplus
}
#endif

#endif  // THROTTLE_MSGS__MSG__DETAIL__THROTTLE_DATA__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
