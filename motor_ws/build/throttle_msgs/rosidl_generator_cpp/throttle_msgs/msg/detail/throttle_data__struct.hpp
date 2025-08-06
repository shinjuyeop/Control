// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from throttle_msgs:msg/ThrottleData.idl
// generated code does not contain a copyright notice

#ifndef THROTTLE_MSGS__MSG__DETAIL__THROTTLE_DATA__STRUCT_HPP_
#define THROTTLE_MSGS__MSG__DETAIL__THROTTLE_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__throttle_msgs__msg__ThrottleData __attribute__((deprecated))
#else
# define DEPRECATED__throttle_msgs__msg__ThrottleData __declspec(deprecated)
#endif

namespace throttle_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ThrottleData_
{
  using Type = ThrottleData_<ContainerAllocator>;

  explicit ThrottleData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rpm = 0;
      this->estop = false;
      this->asms = false;
    }
  }

  explicit ThrottleData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rpm = 0;
      this->estop = false;
      this->asms = false;
    }
  }

  // field types and members
  using _rpm_type =
    int16_t;
  _rpm_type rpm;
  using _estop_type =
    bool;
  _estop_type estop;
  using _asms_type =
    bool;
  _asms_type asms;

  // setters for named parameter idiom
  Type & set__rpm(
    const int16_t & _arg)
  {
    this->rpm = _arg;
    return *this;
  }
  Type & set__estop(
    const bool & _arg)
  {
    this->estop = _arg;
    return *this;
  }
  Type & set__asms(
    const bool & _arg)
  {
    this->asms = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    throttle_msgs::msg::ThrottleData_<ContainerAllocator> *;
  using ConstRawPtr =
    const throttle_msgs::msg::ThrottleData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<throttle_msgs::msg::ThrottleData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<throttle_msgs::msg::ThrottleData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      throttle_msgs::msg::ThrottleData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<throttle_msgs::msg::ThrottleData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      throttle_msgs::msg::ThrottleData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<throttle_msgs::msg::ThrottleData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<throttle_msgs::msg::ThrottleData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<throttle_msgs::msg::ThrottleData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__throttle_msgs__msg__ThrottleData
    std::shared_ptr<throttle_msgs::msg::ThrottleData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__throttle_msgs__msg__ThrottleData
    std::shared_ptr<throttle_msgs::msg::ThrottleData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ThrottleData_ & other) const
  {
    if (this->rpm != other.rpm) {
      return false;
    }
    if (this->estop != other.estop) {
      return false;
    }
    if (this->asms != other.asms) {
      return false;
    }
    return true;
  }
  bool operator!=(const ThrottleData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ThrottleData_

// alias to use template instance with default allocator
using ThrottleData =
  throttle_msgs::msg::ThrottleData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace throttle_msgs

#endif  // THROTTLE_MSGS__MSG__DETAIL__THROTTLE_DATA__STRUCT_HPP_
