// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from throttle_msgs:msg/ThrottleData.idl
// generated code does not contain a copyright notice
#include "throttle_msgs/msg/detail/throttle_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
throttle_msgs__msg__ThrottleData__init(throttle_msgs__msg__ThrottleData * msg)
{
  if (!msg) {
    return false;
  }
  // rpm
  // estop
  // asms
  return true;
}

void
throttle_msgs__msg__ThrottleData__fini(throttle_msgs__msg__ThrottleData * msg)
{
  if (!msg) {
    return;
  }
  // rpm
  // estop
  // asms
}

bool
throttle_msgs__msg__ThrottleData__are_equal(const throttle_msgs__msg__ThrottleData * lhs, const throttle_msgs__msg__ThrottleData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // rpm
  if (lhs->rpm != rhs->rpm) {
    return false;
  }
  // estop
  if (lhs->estop != rhs->estop) {
    return false;
  }
  // asms
  if (lhs->asms != rhs->asms) {
    return false;
  }
  return true;
}

bool
throttle_msgs__msg__ThrottleData__copy(
  const throttle_msgs__msg__ThrottleData * input,
  throttle_msgs__msg__ThrottleData * output)
{
  if (!input || !output) {
    return false;
  }
  // rpm
  output->rpm = input->rpm;
  // estop
  output->estop = input->estop;
  // asms
  output->asms = input->asms;
  return true;
}

throttle_msgs__msg__ThrottleData *
throttle_msgs__msg__ThrottleData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  throttle_msgs__msg__ThrottleData * msg = (throttle_msgs__msg__ThrottleData *)allocator.allocate(sizeof(throttle_msgs__msg__ThrottleData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(throttle_msgs__msg__ThrottleData));
  bool success = throttle_msgs__msg__ThrottleData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
throttle_msgs__msg__ThrottleData__destroy(throttle_msgs__msg__ThrottleData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    throttle_msgs__msg__ThrottleData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
throttle_msgs__msg__ThrottleData__Sequence__init(throttle_msgs__msg__ThrottleData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  throttle_msgs__msg__ThrottleData * data = NULL;

  if (size) {
    data = (throttle_msgs__msg__ThrottleData *)allocator.zero_allocate(size, sizeof(throttle_msgs__msg__ThrottleData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = throttle_msgs__msg__ThrottleData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        throttle_msgs__msg__ThrottleData__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
throttle_msgs__msg__ThrottleData__Sequence__fini(throttle_msgs__msg__ThrottleData__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      throttle_msgs__msg__ThrottleData__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

throttle_msgs__msg__ThrottleData__Sequence *
throttle_msgs__msg__ThrottleData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  throttle_msgs__msg__ThrottleData__Sequence * array = (throttle_msgs__msg__ThrottleData__Sequence *)allocator.allocate(sizeof(throttle_msgs__msg__ThrottleData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = throttle_msgs__msg__ThrottleData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
throttle_msgs__msg__ThrottleData__Sequence__destroy(throttle_msgs__msg__ThrottleData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    throttle_msgs__msg__ThrottleData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
throttle_msgs__msg__ThrottleData__Sequence__are_equal(const throttle_msgs__msg__ThrottleData__Sequence * lhs, const throttle_msgs__msg__ThrottleData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!throttle_msgs__msg__ThrottleData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
throttle_msgs__msg__ThrottleData__Sequence__copy(
  const throttle_msgs__msg__ThrottleData__Sequence * input,
  throttle_msgs__msg__ThrottleData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(throttle_msgs__msg__ThrottleData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    throttle_msgs__msg__ThrottleData * data =
      (throttle_msgs__msg__ThrottleData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!throttle_msgs__msg__ThrottleData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          throttle_msgs__msg__ThrottleData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!throttle_msgs__msg__ThrottleData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
