// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from kaiev26_msgs:msg/Centerline.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__CENTERLINE__FUNCTIONS_H_
#define KAIEV26_MSGS__MSG__DETAIL__CENTERLINE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "kaiev26_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "kaiev26_msgs/msg/detail/centerline__struct.h"

/// Initialize msg/Centerline message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * kaiev26_msgs__msg__Centerline
 * )) before or use
 * kaiev26_msgs__msg__Centerline__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__Centerline__init(kaiev26_msgs__msg__Centerline * msg);

/// Finalize msg/Centerline message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
void
kaiev26_msgs__msg__Centerline__fini(kaiev26_msgs__msg__Centerline * msg);

/// Create msg/Centerline message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * kaiev26_msgs__msg__Centerline__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
kaiev26_msgs__msg__Centerline *
kaiev26_msgs__msg__Centerline__create();

/// Destroy msg/Centerline message.
/**
 * It calls
 * kaiev26_msgs__msg__Centerline__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
void
kaiev26_msgs__msg__Centerline__destroy(kaiev26_msgs__msg__Centerline * msg);

/// Check for msg/Centerline message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__Centerline__are_equal(const kaiev26_msgs__msg__Centerline * lhs, const kaiev26_msgs__msg__Centerline * rhs);

/// Copy a msg/Centerline message.
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
kaiev26_msgs__msg__Centerline__copy(
  const kaiev26_msgs__msg__Centerline * input,
  kaiev26_msgs__msg__Centerline * output);

/// Initialize array of msg/Centerline messages.
/**
 * It allocates the memory for the number of elements and calls
 * kaiev26_msgs__msg__Centerline__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__Centerline__Sequence__init(kaiev26_msgs__msg__Centerline__Sequence * array, size_t size);

/// Finalize array of msg/Centerline messages.
/**
 * It calls
 * kaiev26_msgs__msg__Centerline__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
void
kaiev26_msgs__msg__Centerline__Sequence__fini(kaiev26_msgs__msg__Centerline__Sequence * array);

/// Create array of msg/Centerline messages.
/**
 * It allocates the memory for the array and calls
 * kaiev26_msgs__msg__Centerline__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
kaiev26_msgs__msg__Centerline__Sequence *
kaiev26_msgs__msg__Centerline__Sequence__create(size_t size);

/// Destroy array of msg/Centerline messages.
/**
 * It calls
 * kaiev26_msgs__msg__Centerline__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
void
kaiev26_msgs__msg__Centerline__Sequence__destroy(kaiev26_msgs__msg__Centerline__Sequence * array);

/// Check for msg/Centerline message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__Centerline__Sequence__are_equal(const kaiev26_msgs__msg__Centerline__Sequence * lhs, const kaiev26_msgs__msg__Centerline__Sequence * rhs);

/// Copy an array of msg/Centerline messages.
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
kaiev26_msgs__msg__Centerline__Sequence__copy(
  const kaiev26_msgs__msg__Centerline__Sequence * input,
  kaiev26_msgs__msg__Centerline__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__CENTERLINE__FUNCTIONS_H_
