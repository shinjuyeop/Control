// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from throttle_msgs:msg/ThrottleData.idl
// generated code does not contain a copyright notice

#ifndef THROTTLE_MSGS__MSG__DETAIL__THROTTLE_DATA__FUNCTIONS_H_
#define THROTTLE_MSGS__MSG__DETAIL__THROTTLE_DATA__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "throttle_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "throttle_msgs/msg/detail/throttle_data__struct.h"

/// Initialize msg/ThrottleData message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * throttle_msgs__msg__ThrottleData
 * )) before or use
 * throttle_msgs__msg__ThrottleData__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_throttle_msgs
bool
throttle_msgs__msg__ThrottleData__init(throttle_msgs__msg__ThrottleData * msg);

/// Finalize msg/ThrottleData message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_throttle_msgs
void
throttle_msgs__msg__ThrottleData__fini(throttle_msgs__msg__ThrottleData * msg);

/// Create msg/ThrottleData message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * throttle_msgs__msg__ThrottleData__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_throttle_msgs
throttle_msgs__msg__ThrottleData *
throttle_msgs__msg__ThrottleData__create();

/// Destroy msg/ThrottleData message.
/**
 * It calls
 * throttle_msgs__msg__ThrottleData__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_throttle_msgs
void
throttle_msgs__msg__ThrottleData__destroy(throttle_msgs__msg__ThrottleData * msg);

/// Check for msg/ThrottleData message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_throttle_msgs
bool
throttle_msgs__msg__ThrottleData__are_equal(const throttle_msgs__msg__ThrottleData * lhs, const throttle_msgs__msg__ThrottleData * rhs);

/// Copy a msg/ThrottleData message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_throttle_msgs
bool
throttle_msgs__msg__ThrottleData__copy(
  const throttle_msgs__msg__ThrottleData * input,
  throttle_msgs__msg__ThrottleData * output);

/// Initialize array of msg/ThrottleData messages.
/**
 * It allocates the memory for the number of elements and calls
 * throttle_msgs__msg__ThrottleData__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_throttle_msgs
bool
throttle_msgs__msg__ThrottleData__Sequence__init(throttle_msgs__msg__ThrottleData__Sequence * array, size_t size);

/// Finalize array of msg/ThrottleData messages.
/**
 * It calls
 * throttle_msgs__msg__ThrottleData__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_throttle_msgs
void
throttle_msgs__msg__ThrottleData__Sequence__fini(throttle_msgs__msg__ThrottleData__Sequence * array);

/// Create array of msg/ThrottleData messages.
/**
 * It allocates the memory for the array and calls
 * throttle_msgs__msg__ThrottleData__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_throttle_msgs
throttle_msgs__msg__ThrottleData__Sequence *
throttle_msgs__msg__ThrottleData__Sequence__create(size_t size);

/// Destroy array of msg/ThrottleData messages.
/**
 * It calls
 * throttle_msgs__msg__ThrottleData__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_throttle_msgs
void
throttle_msgs__msg__ThrottleData__Sequence__destroy(throttle_msgs__msg__ThrottleData__Sequence * array);

/// Check for msg/ThrottleData message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_throttle_msgs
bool
throttle_msgs__msg__ThrottleData__Sequence__are_equal(const throttle_msgs__msg__ThrottleData__Sequence * lhs, const throttle_msgs__msg__ThrottleData__Sequence * rhs);

/// Copy an array of msg/ThrottleData messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_throttle_msgs
bool
throttle_msgs__msg__ThrottleData__Sequence__copy(
  const throttle_msgs__msg__ThrottleData__Sequence * input,
  throttle_msgs__msg__ThrottleData__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // THROTTLE_MSGS__MSG__DETAIL__THROTTLE_DATA__FUNCTIONS_H_
