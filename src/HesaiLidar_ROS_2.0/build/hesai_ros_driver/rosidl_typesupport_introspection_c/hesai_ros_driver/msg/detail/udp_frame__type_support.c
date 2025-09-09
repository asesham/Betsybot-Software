// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from hesai_ros_driver:msg/UdpFrame.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "hesai_ros_driver/msg/detail/udp_frame__rosidl_typesupport_introspection_c.h"
#include "hesai_ros_driver/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "hesai_ros_driver/msg/detail/udp_frame__functions.h"
#include "hesai_ros_driver/msg/detail/udp_frame__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `packets`
#include "hesai_ros_driver/msg/udp_packet.h"
// Member `packets`
#include "hesai_ros_driver/msg/detail/udp_packet__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__UdpFrame_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  hesai_ros_driver__msg__UdpFrame__init(message_memory);
}

void hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__UdpFrame_fini_function(void * message_memory)
{
  hesai_ros_driver__msg__UdpFrame__fini(message_memory);
}

size_t hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__size_function__UdpFrame__packets(
  const void * untyped_member)
{
  const hesai_ros_driver__msg__UdpPacket__Sequence * member =
    (const hesai_ros_driver__msg__UdpPacket__Sequence *)(untyped_member);
  return member->size;
}

const void * hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__get_const_function__UdpFrame__packets(
  const void * untyped_member, size_t index)
{
  const hesai_ros_driver__msg__UdpPacket__Sequence * member =
    (const hesai_ros_driver__msg__UdpPacket__Sequence *)(untyped_member);
  return &member->data[index];
}

void * hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__get_function__UdpFrame__packets(
  void * untyped_member, size_t index)
{
  hesai_ros_driver__msg__UdpPacket__Sequence * member =
    (hesai_ros_driver__msg__UdpPacket__Sequence *)(untyped_member);
  return &member->data[index];
}

void hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__fetch_function__UdpFrame__packets(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const hesai_ros_driver__msg__UdpPacket * item =
    ((const hesai_ros_driver__msg__UdpPacket *)
    hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__get_const_function__UdpFrame__packets(untyped_member, index));
  hesai_ros_driver__msg__UdpPacket * value =
    (hesai_ros_driver__msg__UdpPacket *)(untyped_value);
  *value = *item;
}

void hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__assign_function__UdpFrame__packets(
  void * untyped_member, size_t index, const void * untyped_value)
{
  hesai_ros_driver__msg__UdpPacket * item =
    ((hesai_ros_driver__msg__UdpPacket *)
    hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__get_function__UdpFrame__packets(untyped_member, index));
  const hesai_ros_driver__msg__UdpPacket * value =
    (const hesai_ros_driver__msg__UdpPacket *)(untyped_value);
  *item = *value;
}

bool hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__resize_function__UdpFrame__packets(
  void * untyped_member, size_t size)
{
  hesai_ros_driver__msg__UdpPacket__Sequence * member =
    (hesai_ros_driver__msg__UdpPacket__Sequence *)(untyped_member);
  hesai_ros_driver__msg__UdpPacket__Sequence__fini(member);
  return hesai_ros_driver__msg__UdpPacket__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__UdpFrame_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hesai_ros_driver__msg__UdpFrame, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "packets",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hesai_ros_driver__msg__UdpFrame, packets),  // bytes offset in struct
    NULL,  // default value
    hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__size_function__UdpFrame__packets,  // size() function pointer
    hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__get_const_function__UdpFrame__packets,  // get_const(index) function pointer
    hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__get_function__UdpFrame__packets,  // get(index) function pointer
    hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__fetch_function__UdpFrame__packets,  // fetch(index, &value) function pointer
    hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__assign_function__UdpFrame__packets,  // assign(index, value) function pointer
    hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__resize_function__UdpFrame__packets  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__UdpFrame_message_members = {
  "hesai_ros_driver__msg",  // message namespace
  "UdpFrame",  // message name
  2,  // number of fields
  sizeof(hesai_ros_driver__msg__UdpFrame),
  hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__UdpFrame_message_member_array,  // message members
  hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__UdpFrame_init_function,  // function to initialize message memory (memory has to be allocated)
  hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__UdpFrame_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__UdpFrame_message_type_support_handle = {
  0,
  &hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__UdpFrame_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_hesai_ros_driver
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hesai_ros_driver, msg, UdpFrame)() {
  hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__UdpFrame_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__UdpFrame_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, hesai_ros_driver, msg, UdpPacket)();
  if (!hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__UdpFrame_message_type_support_handle.typesupport_identifier) {
    hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__UdpFrame_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &hesai_ros_driver__msg__UdpFrame__rosidl_typesupport_introspection_c__UdpFrame_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
