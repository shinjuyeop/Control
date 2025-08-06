// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from throttle_msgs:msg/ThrottleData.idl
// generated code does not contain a copyright notice

#ifndef THROTTLE_MSGS__MSG__DETAIL__THROTTLE_DATA__BUILDER_HPP_
#define THROTTLE_MSGS__MSG__DETAIL__THROTTLE_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "throttle_msgs/msg/detail/throttle_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace throttle_msgs
{

namespace msg
{

namespace builder
{

class Init_ThrottleData_asms
{
public:
  explicit Init_ThrottleData_asms(::throttle_msgs::msg::ThrottleData & msg)
  : msg_(msg)
  {}
  ::throttle_msgs::msg::ThrottleData asms(::throttle_msgs::msg::ThrottleData::_asms_type arg)
  {
    msg_.asms = std::move(arg);
    return std::move(msg_);
  }

private:
  ::throttle_msgs::msg::ThrottleData msg_;
};

class Init_ThrottleData_estop
{
public:
  explicit Init_ThrottleData_estop(::throttle_msgs::msg::ThrottleData & msg)
  : msg_(msg)
  {}
  Init_ThrottleData_asms estop(::throttle_msgs::msg::ThrottleData::_estop_type arg)
  {
    msg_.estop = std::move(arg);
    return Init_ThrottleData_asms(msg_);
  }

private:
  ::throttle_msgs::msg::ThrottleData msg_;
};

class Init_ThrottleData_rpm
{
public:
  Init_ThrottleData_rpm()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ThrottleData_estop rpm(::throttle_msgs::msg::ThrottleData::_rpm_type arg)
  {
    msg_.rpm = std::move(arg);
    return Init_ThrottleData_estop(msg_);
  }

private:
  ::throttle_msgs::msg::ThrottleData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::throttle_msgs::msg::ThrottleData>()
{
  return throttle_msgs::msg::builder::Init_ThrottleData_rpm();
}

}  // namespace throttle_msgs

#endif  // THROTTLE_MSGS__MSG__DETAIL__THROTTLE_DATA__BUILDER_HPP_
