// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from kaiev26_msgs:msg/RoadSegmentArray.idl
// generated code does not contain a copyright notice

#ifndef KAIEV26_MSGS__MSG__DETAIL__ROAD_SEGMENT_ARRAY__FUNCTIONS_H_
#define KAIEV26_MSGS__MSG__DETAIL__ROAD_SEGMENT_ARRAY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "kaiev26_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "kaiev26_msgs/msg/detail/road_segment_array__struct.h"

/// Initialize msg/RoadSegmentArray message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * kaiev26_msgs__msg__RoadSegmentArray
 * )) before or use
 * kaiev26_msgs__msg__RoadSegmentArray__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__RoadSegmentArray__init(kaiev26_msgs__msg__RoadSegmentArray * msg);

/// Finalize msg/RoadSegmentArray message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
void
kaiev26_msgs__msg__RoadSegmentArray__fini(kaiev26_msgs__msg__RoadSegmentArray * msg);

/// Create msg/RoadSegmentArray message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * kaiev26_msgs__msg__RoadSegmentArray__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
kaiev26_msgs__msg__RoadSegmentArray *
kaiev26_msgs__msg__RoadSegmentArray__create();

/// Destroy msg/RoadSegmentArray message.
/**
 * It calls
 * kaiev26_msgs__msg__RoadSegmentArray__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
void
kaiev26_msgs__msg__RoadSegmentArray__destroy(kaiev26_msgs__msg__RoadSegmentArray * msg);

/// Check for msg/RoadSegmentArray message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__RoadSegmentArray__are_equal(const kaiev26_msgs__msg__RoadSegmentArray * lhs, const kaiev26_msgs__msg__RoadSegmentArray * rhs);

/// Copy a msg/RoadSegmentArray message.
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
kaiev26_msgs__msg__RoadSegmentArray__copy(
  const kaiev26_msgs__msg__RoadSegmentArray * input,
  kaiev26_msgs__msg__RoadSegmentArray * output);

/// Initialize array of msg/RoadSegmentArray messages.
/**
 * It allocates the memory for the number of elements and calls
 * kaiev26_msgs__msg__RoadSegmentArray__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__RoadSegmentArray__Sequence__init(kaiev26_msgs__msg__RoadSegmentArray__Sequence * array, size_t size);

/// Finalize array of msg/RoadSegmentArray messages.
/**
 * It calls
 * kaiev26_msgs__msg__RoadSegmentArray__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
void
kaiev26_msgs__msg__RoadSegmentArray__Sequence__fini(kaiev26_msgs__msg__RoadSegmentArray__Sequence * array);

/// Create array of msg/RoadSegmentArray messages.
/**
 * It allocates the memory for the array and calls
 * kaiev26_msgs__msg__RoadSegmentArray__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
kaiev26_msgs__msg__RoadSegmentArray__Sequence *
kaiev26_msgs__msg__RoadSegmentArray__Sequence__create(size_t size);

/// Destroy array of msg/RoadSegmentArray messages.
/**
 * It calls
 * kaiev26_msgs__msg__RoadSegmentArray__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
void
kaiev26_msgs__msg__RoadSegmentArray__Sequence__destroy(kaiev26_msgs__msg__RoadSegmentArray__Sequence * array);

/// Check for msg/RoadSegmentArray message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_kaiev26_msgs
bool
kaiev26_msgs__msg__RoadSegmentArray__Sequence__are_equal(const kaiev26_msgs__msg__RoadSegmentArray__Sequence * lhs, const kaiev26_msgs__msg__RoadSegmentArray__Sequence * rhs);

/// Copy an array of msg/RoadSegmentArray messages.
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
kaiev26_msgs__msg__RoadSegmentArray__Sequence__copy(
  const kaiev26_msgs__msg__RoadSegmentArray__Sequence * input,
  kaiev26_msgs__msg__RoadSegmentArray__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // KAIEV26_MSGS__MSG__DETAIL__ROAD_SEGMENT_ARRAY__FUNCTIONS_H_
