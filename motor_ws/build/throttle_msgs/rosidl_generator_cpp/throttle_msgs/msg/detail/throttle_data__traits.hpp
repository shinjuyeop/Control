// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from throttle_msgs:msg/ThrottleData.idl
// generated code does not contain a copyright notice

#ifndef THROTTLE_MSGS__MSG__DETAIL__THROTTLE_DATA__TRAITS_HPP_
#define THROTTLE_MSGS__MSG__DETAIL__THROTTLE_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "throttle_msgs/msg/detail/throttle_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace throttle_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ThrottleData & msg,
  std::ostream & out)
{
  out << "{";
  // member: rpm
  {
    out << "rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.rpm, out);
    out << ", ";
  }

  // member: estop
  {
    out << "estop: ";
    rosidl_generator_traits::value_to_yaml(msg.estop, out);
    out << ", ";
  }

  // member: asms
  {
    out << "asms: ";
    rosidl_generator_traits::value_to_yaml(msg.asms, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ThrottleData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: rpm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.rpm, out);
    out << "\n";
  }

  // member: estop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "estop: ";
    rosidl_generator_traits::value_to_yaml(msg.estop, out);
    out << "\n";
  }

  // member: asms
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "asms: ";
    rosidl_generator_traits::value_to_yaml(msg.asms, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ThrottleData & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace throttle_msgs

namespace rosidl_generator_traits
{

[[deprecated("use throttle_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const throttle_msgs::msg::ThrottleData & msg,
  std::ostream & out, size_t indentation = 0)
{
  throttle_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use throttle_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const throttle_msgs::msg::ThrottleData & msg)
{
  return throttle_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<throttle_msgs::msg::ThrottleData>()
{
  return "throttle_msgs::msg::ThrottleData";
}

template<>
inline const char * name<throttle_msgs::msg::ThrottleData>()
{
  return "throttle_msgs/msg/ThrottleData";
}

template<>
struct has_fixed_size<throttle_msgs::msg::ThrottleData>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<throttle_msgs::msg::ThrottleData>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<throttle_msgs::msg::ThrottleData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // THROTTLE_MSGS__MSG__DETAIL__THROTTLE_DATA__TRAITS_HPP_
