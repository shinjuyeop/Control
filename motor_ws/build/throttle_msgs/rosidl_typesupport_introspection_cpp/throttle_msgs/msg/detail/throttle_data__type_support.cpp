// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from throttle_msgs:msg/ThrottleData.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "throttle_msgs/msg/detail/throttle_data__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace throttle_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ThrottleData_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) throttle_msgs::msg::ThrottleData(_init);
}

void ThrottleData_fini_function(void * message_memory)
{
  auto typed_message = static_cast<throttle_msgs::msg::ThrottleData *>(message_memory);
  typed_message->~ThrottleData();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ThrottleData_message_member_array[3] = {
  {
    "rpm",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(throttle_msgs::msg::ThrottleData, rpm),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "estop",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(throttle_msgs::msg::ThrottleData, estop),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "asms",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(throttle_msgs::msg::ThrottleData, asms),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ThrottleData_message_members = {
  "throttle_msgs::msg",  // message namespace
  "ThrottleData",  // message name
  3,  // number of fields
  sizeof(throttle_msgs::msg::ThrottleData),
  ThrottleData_message_member_array,  // message members
  ThrottleData_init_function,  // function to initialize message memory (memory has to be allocated)
  ThrottleData_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ThrottleData_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ThrottleData_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace throttle_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<throttle_msgs::msg::ThrottleData>()
{
  return &::throttle_msgs::msg::rosidl_typesupport_introspection_cpp::ThrottleData_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, throttle_msgs, msg, ThrottleData)() {
  return &::throttle_msgs::msg::rosidl_typesupport_introspection_cpp::ThrottleData_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
