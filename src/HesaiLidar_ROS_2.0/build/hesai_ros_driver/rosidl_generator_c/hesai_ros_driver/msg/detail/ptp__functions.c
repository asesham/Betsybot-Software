// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hesai_ros_driver:msg/Ptp.idl
// generated code does not contain a copyright notice
#include "hesai_ros_driver/msg/detail/ptp__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
hesai_ros_driver__msg__Ptp__init(hesai_ros_driver__msg__Ptp * msg)
{
  if (!msg) {
    return false;
  }
  // ptp_lock_offset
  // ptp_status
  return true;
}

void
hesai_ros_driver__msg__Ptp__fini(hesai_ros_driver__msg__Ptp * msg)
{
  if (!msg) {
    return;
  }
  // ptp_lock_offset
  // ptp_status
}

bool
hesai_ros_driver__msg__Ptp__are_equal(const hesai_ros_driver__msg__Ptp * lhs, const hesai_ros_driver__msg__Ptp * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ptp_lock_offset
  if (lhs->ptp_lock_offset != rhs->ptp_lock_offset) {
    return false;
  }
  // ptp_status
  for (size_t i = 0; i < 16; ++i) {
    if (lhs->ptp_status[i] != rhs->ptp_status[i]) {
      return false;
    }
  }
  return true;
}

bool
hesai_ros_driver__msg__Ptp__copy(
  const hesai_ros_driver__msg__Ptp * input,
  hesai_ros_driver__msg__Ptp * output)
{
  if (!input || !output) {
    return false;
  }
  // ptp_lock_offset
  output->ptp_lock_offset = input->ptp_lock_offset;
  // ptp_status
  for (size_t i = 0; i < 16; ++i) {
    output->ptp_status[i] = input->ptp_status[i];
  }
  return true;
}

hesai_ros_driver__msg__Ptp *
hesai_ros_driver__msg__Ptp__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hesai_ros_driver__msg__Ptp * msg = (hesai_ros_driver__msg__Ptp *)allocator.allocate(sizeof(hesai_ros_driver__msg__Ptp), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hesai_ros_driver__msg__Ptp));
  bool success = hesai_ros_driver__msg__Ptp__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hesai_ros_driver__msg__Ptp__destroy(hesai_ros_driver__msg__Ptp * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hesai_ros_driver__msg__Ptp__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hesai_ros_driver__msg__Ptp__Sequence__init(hesai_ros_driver__msg__Ptp__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hesai_ros_driver__msg__Ptp * data = NULL;

  if (size) {
    data = (hesai_ros_driver__msg__Ptp *)allocator.zero_allocate(size, sizeof(hesai_ros_driver__msg__Ptp), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hesai_ros_driver__msg__Ptp__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hesai_ros_driver__msg__Ptp__fini(&data[i - 1]);
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
hesai_ros_driver__msg__Ptp__Sequence__fini(hesai_ros_driver__msg__Ptp__Sequence * array)
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
      hesai_ros_driver__msg__Ptp__fini(&array->data[i]);
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

hesai_ros_driver__msg__Ptp__Sequence *
hesai_ros_driver__msg__Ptp__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hesai_ros_driver__msg__Ptp__Sequence * array = (hesai_ros_driver__msg__Ptp__Sequence *)allocator.allocate(sizeof(hesai_ros_driver__msg__Ptp__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hesai_ros_driver__msg__Ptp__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hesai_ros_driver__msg__Ptp__Sequence__destroy(hesai_ros_driver__msg__Ptp__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hesai_ros_driver__msg__Ptp__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hesai_ros_driver__msg__Ptp__Sequence__are_equal(const hesai_ros_driver__msg__Ptp__Sequence * lhs, const hesai_ros_driver__msg__Ptp__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hesai_ros_driver__msg__Ptp__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hesai_ros_driver__msg__Ptp__Sequence__copy(
  const hesai_ros_driver__msg__Ptp__Sequence * input,
  hesai_ros_driver__msg__Ptp__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hesai_ros_driver__msg__Ptp);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hesai_ros_driver__msg__Ptp * data =
      (hesai_ros_driver__msg__Ptp *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hesai_ros_driver__msg__Ptp__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hesai_ros_driver__msg__Ptp__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hesai_ros_driver__msg__Ptp__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
