// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hesai_ros_driver:msg/Firetime.idl
// generated code does not contain a copyright notice
#include "hesai_ros_driver/msg/detail/firetime__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
hesai_ros_driver__msg__Firetime__init(hesai_ros_driver__msg__Firetime * msg)
{
  if (!msg) {
    return false;
  }
  // data
  return true;
}

void
hesai_ros_driver__msg__Firetime__fini(hesai_ros_driver__msg__Firetime * msg)
{
  if (!msg) {
    return;
  }
  // data
}

bool
hesai_ros_driver__msg__Firetime__are_equal(const hesai_ros_driver__msg__Firetime * lhs, const hesai_ros_driver__msg__Firetime * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // data
  for (size_t i = 0; i < 512; ++i) {
    if (lhs->data[i] != rhs->data[i]) {
      return false;
    }
  }
  return true;
}

bool
hesai_ros_driver__msg__Firetime__copy(
  const hesai_ros_driver__msg__Firetime * input,
  hesai_ros_driver__msg__Firetime * output)
{
  if (!input || !output) {
    return false;
  }
  // data
  for (size_t i = 0; i < 512; ++i) {
    output->data[i] = input->data[i];
  }
  return true;
}

hesai_ros_driver__msg__Firetime *
hesai_ros_driver__msg__Firetime__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hesai_ros_driver__msg__Firetime * msg = (hesai_ros_driver__msg__Firetime *)allocator.allocate(sizeof(hesai_ros_driver__msg__Firetime), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hesai_ros_driver__msg__Firetime));
  bool success = hesai_ros_driver__msg__Firetime__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hesai_ros_driver__msg__Firetime__destroy(hesai_ros_driver__msg__Firetime * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hesai_ros_driver__msg__Firetime__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hesai_ros_driver__msg__Firetime__Sequence__init(hesai_ros_driver__msg__Firetime__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hesai_ros_driver__msg__Firetime * data = NULL;

  if (size) {
    data = (hesai_ros_driver__msg__Firetime *)allocator.zero_allocate(size, sizeof(hesai_ros_driver__msg__Firetime), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hesai_ros_driver__msg__Firetime__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hesai_ros_driver__msg__Firetime__fini(&data[i - 1]);
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
hesai_ros_driver__msg__Firetime__Sequence__fini(hesai_ros_driver__msg__Firetime__Sequence * array)
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
      hesai_ros_driver__msg__Firetime__fini(&array->data[i]);
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

hesai_ros_driver__msg__Firetime__Sequence *
hesai_ros_driver__msg__Firetime__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hesai_ros_driver__msg__Firetime__Sequence * array = (hesai_ros_driver__msg__Firetime__Sequence *)allocator.allocate(sizeof(hesai_ros_driver__msg__Firetime__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hesai_ros_driver__msg__Firetime__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hesai_ros_driver__msg__Firetime__Sequence__destroy(hesai_ros_driver__msg__Firetime__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hesai_ros_driver__msg__Firetime__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hesai_ros_driver__msg__Firetime__Sequence__are_equal(const hesai_ros_driver__msg__Firetime__Sequence * lhs, const hesai_ros_driver__msg__Firetime__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hesai_ros_driver__msg__Firetime__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hesai_ros_driver__msg__Firetime__Sequence__copy(
  const hesai_ros_driver__msg__Firetime__Sequence * input,
  hesai_ros_driver__msg__Firetime__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hesai_ros_driver__msg__Firetime);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hesai_ros_driver__msg__Firetime * data =
      (hesai_ros_driver__msg__Firetime *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hesai_ros_driver__msg__Firetime__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hesai_ros_driver__msg__Firetime__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hesai_ros_driver__msg__Firetime__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
