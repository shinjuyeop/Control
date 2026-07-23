// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from kaiev26_msgs:msg/ManualDriveInput.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__MANUAL_DRIVE_INPUT__FUNCTIONS_H_
#define KAIEV26_MSGS__MSG__DETAIL__MANUAL_DRIVE_INPUT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "kaiev26_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "kaiev26_msgs/msg/detail/manual_drive_input__struct.h"

/// Initialize msg/ManualDriveInput message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * kaiev26_msgs__msg__ManualDriveInput
 * )) before or use
 * kaiev26_msgs__msg__ManualDriveInput__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__ManualDriveInput__init(kaiev26_msgs__msg__ManualDriveInput * msg);

/// Finalize msg/ManualDriveInput message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
void
kaiev26_msgs__msg__ManualDriveInput__fini(kaiev26_msgs__msg__ManualDriveInput * msg);

/// Create msg/ManualDriveInput message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * kaiev26_msgs__msg__ManualDriveInput__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
kaiev26_msgs__msg__ManualDriveInput *
kaiev26_msgs__msg__ManualDriveInput__create();

/// Destroy msg/ManualDriveInput message.
/**
 * It calls
 * kaiev26_msgs__msg__ManualDriveInput__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
void
kaiev26_msgs__msg__ManualDriveInput__destroy(kaiev26_msgs__msg__ManualDriveInput * msg);

/// Check for msg/ManualDriveInput message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__ManualDriveInput__are_equal(const kaiev26_msgs__msg__ManualDriveInput * lhs, const kaiev26_msgs__msg__ManualDriveInput * rhs);

/// Copy a msg/ManualDriveInput message.
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
kaiev26_msgs__msg__ManualDriveInput__copy(
  const kaiev26_msgs__msg__ManualDriveInput * input,
  kaiev26_msgs__msg__ManualDriveInput * output);

/// Initialize array of msg/ManualDriveInput messages.
/**
 * It allocates the memory for the number of elements and calls
 * kaiev26_msgs__msg__ManualDriveInput__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__ManualDriveInput__Sequence__init(kaiev26_msgs__msg__ManualDriveInput__Sequence * array, size_t size);

/// Finalize array of msg/ManualDriveInput messages.
/**
 * It calls
 * kaiev26_msgs__msg__ManualDriveInput__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
void
kaiev26_msgs__msg__ManualDriveInput__Sequence__fini(kaiev26_msgs__msg__ManualDriveInput__Sequence * array);

/// Create array of msg/ManualDriveInput messages.
/**
 * It allocates the memory for the array and calls
 * kaiev26_msgs__msg__ManualDriveInput__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
kaiev26_msgs__msg__ManualDriveInput__Sequence *
kaiev26_msgs__msg__ManualDriveInput__Sequence__create(size_t size);

/// Destroy array of msg/ManualDriveInput messages.
/**
 * It calls
 * kaiev26_msgs__msg__ManualDriveInput__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
void
kaiev26_msgs__msg__ManualDriveInput__Sequence__destroy(kaiev26_msgs__msg__ManualDriveInput__Sequence * array);

/// Check for msg/ManualDriveInput message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__ManualDriveInput__Sequence__are_equal(const kaiev26_msgs__msg__ManualDriveInput__Sequence * lhs, const kaiev26_msgs__msg__ManualDriveInput__Sequence * rhs);

/// Copy an array of msg/ManualDriveInput messages.
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
kaiev26_msgs__msg__ManualDriveInput__Sequence__copy(
  const kaiev26_msgs__msg__ManualDriveInput__Sequence * input,
  kaiev26_msgs__msg__ManualDriveInput__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__MANUAL_DRIVE_INPUT__FUNCTIONS_H_
