// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from kaiev26_msgs:msg/Perception2DFrame.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__PERCEPTION2_D_FRAME__FUNCTIONS_H_
#define KAIEV26_MSGS__MSG__DETAIL__PERCEPTION2_D_FRAME__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "kaiev26_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "kaiev26_msgs/msg/detail/perception2_d_frame__struct.h"

/// Initialize msg/Perception2DFrame message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * kaiev26_msgs__msg__Perception2DFrame
 * )) before or use
 * kaiev26_msgs__msg__Perception2DFrame__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__Perception2DFrame__init(kaiev26_msgs__msg__Perception2DFrame * msg);

/// Finalize msg/Perception2DFrame message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
void
kaiev26_msgs__msg__Perception2DFrame__fini(kaiev26_msgs__msg__Perception2DFrame * msg);

/// Create msg/Perception2DFrame message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * kaiev26_msgs__msg__Perception2DFrame__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
kaiev26_msgs__msg__Perception2DFrame *
kaiev26_msgs__msg__Perception2DFrame__create();

/// Destroy msg/Perception2DFrame message.
/**
 * It calls
 * kaiev26_msgs__msg__Perception2DFrame__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
void
kaiev26_msgs__msg__Perception2DFrame__destroy(kaiev26_msgs__msg__Perception2DFrame * msg);

/// Check for msg/Perception2DFrame message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__Perception2DFrame__are_equal(const kaiev26_msgs__msg__Perception2DFrame * lhs, const kaiev26_msgs__msg__Perception2DFrame * rhs);

/// Copy a msg/Perception2DFrame message.
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
kaiev26_msgs__msg__Perception2DFrame__copy(
  const kaiev26_msgs__msg__Perception2DFrame * input,
  kaiev26_msgs__msg__Perception2DFrame * output);

/// Initialize array of msg/Perception2DFrame messages.
/**
 * It allocates the memory for the number of elements and calls
 * kaiev26_msgs__msg__Perception2DFrame__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__Perception2DFrame__Sequence__init(kaiev26_msgs__msg__Perception2DFrame__Sequence * array, size_t size);

/// Finalize array of msg/Perception2DFrame messages.
/**
 * It calls
 * kaiev26_msgs__msg__Perception2DFrame__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
void
kaiev26_msgs__msg__Perception2DFrame__Sequence__fini(kaiev26_msgs__msg__Perception2DFrame__Sequence * array);

/// Create array of msg/Perception2DFrame messages.
/**
 * It allocates the memory for the array and calls
 * kaiev26_msgs__msg__Perception2DFrame__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
kaiev26_msgs__msg__Perception2DFrame__Sequence *
kaiev26_msgs__msg__Perception2DFrame__Sequence__create(size_t size);

/// Destroy array of msg/Perception2DFrame messages.
/**
 * It calls
 * kaiev26_msgs__msg__Perception2DFrame__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
void
kaiev26_msgs__msg__Perception2DFrame__Sequence__destroy(kaiev26_msgs__msg__Perception2DFrame__Sequence * array);

/// Check for msg/Perception2DFrame message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__Perception2DFrame__Sequence__are_equal(const kaiev26_msgs__msg__Perception2DFrame__Sequence * lhs, const kaiev26_msgs__msg__Perception2DFrame__Sequence * rhs);

/// Copy an array of msg/Perception2DFrame messages.
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
kaiev26_msgs__msg__Perception2DFrame__Sequence__copy(
  const kaiev26_msgs__msg__Perception2DFrame__Sequence * input,
  kaiev26_msgs__msg__Perception2DFrame__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__PERCEPTION2_D_FRAME__FUNCTIONS_H_
