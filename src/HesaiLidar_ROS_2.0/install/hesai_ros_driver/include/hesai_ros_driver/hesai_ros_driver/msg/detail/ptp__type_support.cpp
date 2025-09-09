// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from hesai_ros_driver:msg/Ptp.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "hesai_ros_driver/msg/detail/ptp__struct.hpp"
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

void Ptp_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) hesai_ros_driver::msg::Ptp(_init);
}

void Ptp_fini_function(void * message_memory)
{
  auto typed_message = static_cast<hesai_ros_driver::msg::Ptp *>(message_memory);
  typed_message->~Ptp();
}

size_t size_function__Ptp__ptp_status(const void * untyped_member)
{
  (void)untyped_member;
  return 16;
}

const void * get_const_function__Ptp__ptp_status(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<uint8_t, 16> *>(untyped_member);
  return &member[index];
}

void * get_function__Ptp__ptp_status(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<uint8_t, 16> *>(untyped_member);
  return &member[index];
}

void fetch_function__Ptp__ptp_status(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const uint8_t *>(
    get_const_function__Ptp__ptp_status(untyped_member, index));
  auto & value = *reinterpret_cast<uint8_t *>(untyped_value);
  value = item;
}

void assign_function__Ptp__ptp_status(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<uint8_t *>(
    get_function__Ptp__ptp_status(untyped_member, index));
  const auto & value = *reinterpret_cast<const uint8_t *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Ptp_message_member_array[2] = {
  {
    "ptp_lock_offset",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(hesai_ros_driver::msg::Ptp, ptp_lock_offset),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "ptp_status",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    16,  // array size
    false,  // is upper bound
    offsetof(hesai_ros_driver::msg::Ptp, ptp_status),  // bytes offset in struct
    nullptr,  // default value
    size_function__Ptp__ptp_status,  // size() function pointer
    get_const_function__Ptp__ptp_status,  // get_const(index) function pointer
    get_function__Ptp__ptp_status,  // get(index) function pointer
    fetch_function__Ptp__ptp_status,  // fetch(index, &value) function pointer
    assign_function__Ptp__ptp_status,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Ptp_message_members = {
  "hesai_ros_driver::msg",  // message namespace
  "Ptp",  // message name
  2,  // number of fields
  sizeof(hesai_ros_driver::msg::Ptp),
  Ptp_message_member_array,  // message members
  Ptp_init_function,  // function to initialize message memory (memory has to be allocated)
  Ptp_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Ptp_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Ptp_message_members,
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
get_message_type_support_handle<hesai_ros_driver::msg::Ptp>()
{
  return &::hesai_ros_driver::msg::rosidl_typesupport_introspection_cpp::Ptp_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, hesai_ros_driver, msg, Ptp)() {
  return &::hesai_ros_driver::msg::rosidl_typesupport_introspection_cpp::Ptp_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
