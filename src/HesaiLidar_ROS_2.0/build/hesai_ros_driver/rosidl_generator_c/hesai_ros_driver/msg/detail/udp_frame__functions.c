// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from hesai_ros_driver:msg/UdpFrame.idl
// generated code does not contain a copyright notice
#include "hesai_ros_driver/msg/detail/udp_frame__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `packets`
#include "hesai_ros_driver/msg/detail/udp_packet__functions.h"

bool
hesai_ros_driver__msg__UdpFrame__init(hesai_ros_driver__msg__UdpFrame * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    hesai_ros_driver__msg__UdpFrame__fini(msg);
    return false;
  }
  // packets
  if (!hesai_ros_driver__msg__UdpPacket__Sequence__init(&msg->packets, 0)) {
    hesai_ros_driver__msg__UdpFrame__fini(msg);
    return false;
  }
  return true;
}

void
hesai_ros_driver__msg__UdpFrame__fini(hesai_ros_driver__msg__UdpFrame * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // packets
  hesai_ros_driver__msg__UdpPacket__Sequence__fini(&msg->packets);
}

bool
hesai_ros_driver__msg__UdpFrame__are_equal(const hesai_ros_driver__msg__UdpFrame * lhs, const hesai_ros_driver__msg__UdpFrame * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // packets
  if (!hesai_ros_driver__msg__UdpPacket__Sequence__are_equal(
      &(lhs->packets), &(rhs->packets)))
  {
    return false;
  }
  return true;
}

bool
hesai_ros_driver__msg__UdpFrame__copy(
  const hesai_ros_driver__msg__UdpFrame * input,
  hesai_ros_driver__msg__UdpFrame * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // packets
  if (!hesai_ros_driver__msg__UdpPacket__Sequence__copy(
      &(input->packets), &(output->packets)))
  {
    return false;
  }
  return true;
}

hesai_ros_driver__msg__UdpFrame *
hesai_ros_driver__msg__UdpFrame__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hesai_ros_driver__msg__UdpFrame * msg = (hesai_ros_driver__msg__UdpFrame *)allocator.allocate(sizeof(hesai_ros_driver__msg__UdpFrame), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(hesai_ros_driver__msg__UdpFrame));
  bool success = hesai_ros_driver__msg__UdpFrame__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
hesai_ros_driver__msg__UdpFrame__destroy(hesai_ros_driver__msg__UdpFrame * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    hesai_ros_driver__msg__UdpFrame__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
hesai_ros_driver__msg__UdpFrame__Sequence__init(hesai_ros_driver__msg__UdpFrame__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hesai_ros_driver__msg__UdpFrame * data = NULL;

  if (size) {
    data = (hesai_ros_driver__msg__UdpFrame *)allocator.zero_allocate(size, sizeof(hesai_ros_driver__msg__UdpFrame), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = hesai_ros_driver__msg__UdpFrame__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        hesai_ros_driver__msg__UdpFrame__fini(&data[i - 1]);
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
hesai_ros_driver__msg__UdpFrame__Sequence__fini(hesai_ros_driver__msg__UdpFrame__Sequence * array)
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
      hesai_ros_driver__msg__UdpFrame__fini(&array->data[i]);
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

hesai_ros_driver__msg__UdpFrame__Sequence *
hesai_ros_driver__msg__UdpFrame__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  hesai_ros_driver__msg__UdpFrame__Sequence * array = (hesai_ros_driver__msg__UdpFrame__Sequence *)allocator.allocate(sizeof(hesai_ros_driver__msg__UdpFrame__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = hesai_ros_driver__msg__UdpFrame__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
hesai_ros_driver__msg__UdpFrame__Sequence__destroy(hesai_ros_driver__msg__UdpFrame__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    hesai_ros_driver__msg__UdpFrame__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
hesai_ros_driver__msg__UdpFrame__Sequence__are_equal(const hesai_ros_driver__msg__UdpFrame__Sequence * lhs, const hesai_ros_driver__msg__UdpFrame__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!hesai_ros_driver__msg__UdpFrame__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
hesai_ros_driver__msg__UdpFrame__Sequence__copy(
  const hesai_ros_driver__msg__UdpFrame__Sequence * input,
  hesai_ros_driver__msg__UdpFrame__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(hesai_ros_driver__msg__UdpFrame);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    hesai_ros_driver__msg__UdpFrame * data =
      (hesai_ros_driver__msg__UdpFrame *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!hesai_ros_driver__msg__UdpFrame__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          hesai_ros_driver__msg__UdpFrame__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!hesai_ros_driver__msg__UdpFrame__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
