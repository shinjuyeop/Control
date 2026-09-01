// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from kaiev26_msgs:msg/RouteContext.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__ROUTE_CONTEXT__FUNCTIONS_H_
#define KAIEV26_MSGS__MSG__DETAIL__ROUTE_CONTEXT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "kaiev26_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "kaiev26_msgs/msg/detail/route_context__struct.h"

/// Initialize msg/RouteContext message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * kaiev26_msgs__msg__RouteContext
 * )) before or use
 * kaiev26_msgs__msg__RouteContext__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__RouteContext__init(kaiev26_msgs__msg__RouteContext * msg);

/// Finalize msg/RouteContext message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
void
kaiev26_msgs__msg__RouteContext__fini(kaiev26_msgs__msg__RouteContext * msg);

/// Create msg/RouteContext message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * kaiev26_msgs__msg__RouteContext__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
kaiev26_msgs__msg__RouteContext *
kaiev26_msgs__msg__RouteContext__create();

/// Destroy msg/RouteContext message.
/**
 * It calls
 * kaiev26_msgs__msg__RouteContext__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
void
kaiev26_msgs__msg__RouteContext__destroy(kaiev26_msgs__msg__RouteContext * msg);

/// Check for msg/RouteContext message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__RouteContext__are_equal(const kaiev26_msgs__msg__RouteContext * lhs, const kaiev26_msgs__msg__RouteContext * rhs);

/// Copy a msg/RouteContext message.
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
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__RouteContext__copy(
  const kaiev26_msgs__msg__RouteContext * input,
  kaiev26_msgs__msg__RouteContext * output);

/// Initialize array of msg/RouteContext messages.
/**
 * It allocates the memory for the number of elements and calls
 * kaiev26_msgs__msg__RouteContext__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__RouteContext__Sequence__init(kaiev26_msgs__msg__RouteContext__Sequence * array, size_t size);

/// Finalize array of msg/RouteContext messages.
/**
 * It calls
 * kaiev26_msgs__msg__RouteContext__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
void
kaiev26_msgs__msg__RouteContext__Sequence__fini(kaiev26_msgs__msg__RouteContext__Sequence * array);

/// Create array of msg/RouteContext messages.
/**
 * It allocates the memory for the array and calls
 * kaiev26_msgs__msg__RouteContext__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
kaiev26_msgs__msg__RouteContext__Sequence *
kaiev26_msgs__msg__RouteContext__Sequence__create(size_t size);

/// Destroy array of msg/RouteContext messages.
/**
 * It calls
 * kaiev26_msgs__msg__RouteContext__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
void
kaiev26_msgs__msg__RouteContext__Sequence__destroy(kaiev26_msgs__msg__RouteContext__Sequence * array);

/// Check for msg/RouteContext message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__RouteContext__Sequence__are_equal(const kaiev26_msgs__msg__RouteContext__Sequence * lhs, const kaiev26_msgs__msg__RouteContext__Sequence * rhs);

/// Copy an array of msg/RouteContext messages.
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
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__RouteContext__Sequence__copy(
  const kaiev26_msgs__msg__RouteContext__Sequence * input,
  kaiev26_msgs__msg__RouteContext__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__ROUTE_CONTEXT__FUNCTIONS_H_
