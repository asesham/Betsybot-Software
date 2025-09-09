// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from hesai_ros_driver:msg/LossPacket.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "hesai_ros_driver/msg/detail/loss_packet__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace hesai_ros_driver
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void LossPacket_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) hesai_ros_driver::msg::LossPacket(_init);
}

void LossPacket_fini_function(void * message_memory)
{
  auto typed_message = static_cast<hesai_ros_driver::msg::LossPacket *>(message_memory);
  typed_message->~LossPacket();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember LossPacket_message_member_array[2] = {
  {
    "total_packet_count",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hesai_ros_driver::msg::LossPacket, total_packet_count),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "total_packet_loss_count",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hesai_ros_driver::msg::LossPacket, total_packet_loss_count),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers LossPacket_message_members = {
  "hesai_ros_driver::msg",  // message namespace
  "LossPacket",  // message name
  2,  // number of fields
  sizeof(hesai_ros_driver::msg::LossPacket),
  LossPacket_message_member_array,  // message members
  LossPacket_init_function,  // function to initialize message memory (memory has to be allocated)
  LossPacket_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t LossPacket_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &LossPacket_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace hesai_ros_driver


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<hesai_ros_driver::msg::LossPacket>()
{
  return &::hesai_ros_driver::msg::rosidl_typesupport_introspection_cpp::LossPacket_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hesai_ros_driver, msg, LossPacket)() {
  return &::hesai_ros_driver::msg::rosidl_typesupport_introspection_cpp::LossPacket_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
