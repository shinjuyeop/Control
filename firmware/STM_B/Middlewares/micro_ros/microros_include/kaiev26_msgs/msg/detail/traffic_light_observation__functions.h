// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from kaiev26_msgs:msg/TrafficLightObservation.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__TRAFFIC_LIGHT_OBSERVATION__FUNCTIONS_H_
#define KAIEV26_MSGS__MSG__DETAIL__TRAFFIC_LIGHT_OBSERVATION__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "kaiev26_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "kaiev26_msgs/msg/detail/traffic_light_observation__struct.h"

/// Initialize msg/TrafficLightObservation message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * kaiev26_msgs__msg__TrafficLightObservation
 * )) before or use
 * kaiev26_msgs__msg__TrafficLightObservation__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__TrafficLightObservation__init(kaiev26_msgs__msg__TrafficLightObservation * msg);

/// Finalize msg/TrafficLightObservation message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
void
kaiev26_msgs__msg__TrafficLightObservation__fini(kaiev26_msgs__msg__TrafficLightObservation * msg);

/// Create msg/TrafficLightObservation message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * kaiev26_msgs__msg__TrafficLightObservation__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
kaiev26_msgs__msg__TrafficLightObservation *
kaiev26_msgs__msg__TrafficLightObservation__create();

/// Destroy msg/TrafficLightObservation message.
/**
 * It calls
 * kaiev26_msgs__msg__TrafficLightObservation__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
void
kaiev26_msgs__msg__TrafficLightObservation__destroy(kaiev26_msgs__msg__TrafficLightObservation * msg);

/// Check for msg/TrafficLightObservation message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__TrafficLightObservation__are_equal(const kaiev26_msgs__msg__TrafficLightObservation * lhs, const kaiev26_msgs__msg__TrafficLightObservation * rhs);

/// Copy a msg/TrafficLightObservation message.
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
kaiev26_msgs__msg__TrafficLightObservation__copy(
  const kaiev26_msgs__msg__TrafficLightObservation * input,
  kaiev26_msgs__msg__TrafficLightObservation * output);

/// Initialize array of msg/TrafficLightObservation messages.
/**
 * It allocates the memory for the number of elements and calls
 * kaiev26_msgs__msg__TrafficLightObservation__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__TrafficLightObservation__Sequence__init(kaiev26_msgs__msg__TrafficLightObservation__Sequence * array, size_t size);

/// Finalize array of msg/TrafficLightObservation messages.
/**
 * It calls
 * kaiev26_msgs__msg__TrafficLightObservation__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
void
kaiev26_msgs__msg__TrafficLightObservation__Sequence__fini(kaiev26_msgs__msg__TrafficLightObservation__Sequence * array);

/// Create array of msg/TrafficLightObservation messages.
/**
 * It allocates the memory for the array and calls
 * kaiev26_msgs__msg__TrafficLightObservation__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
kaiev26_msgs__msg__TrafficLightObservation__Sequence *
kaiev26_msgs__msg__TrafficLightObservation__Sequence__create(size_t size);

/// Destroy array of msg/TrafficLightObservation messages.
/**
 * It calls
 * kaiev26_msgs__msg__TrafficLightObservation__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
void
kaiev26_msgs__msg__TrafficLightObservation__Sequence__destroy(kaiev26_msgs__msg__TrafficLightObservation__Sequence * array);

/// Check for msg/TrafficLightObservation message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__TrafficLightObservation__Sequence__are_equal(const kaiev26_msgs__msg__TrafficLightObservation__Sequence * lhs, const kaiev26_msgs__msg__TrafficLightObservation__Sequence * rhs);

/// Copy an array of msg/TrafficLightObservation messages.
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
kaiev26_msgs__msg__TrafficLightObservation__Sequence__copy(
  const kaiev26_msgs__msg__TrafficLightObservation__Sequence * input,
  kaiev26_msgs__msg__TrafficLightObservation__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__TRAFFIC_LIGHT_OBSERVATION__FUNCTIONS_H_
