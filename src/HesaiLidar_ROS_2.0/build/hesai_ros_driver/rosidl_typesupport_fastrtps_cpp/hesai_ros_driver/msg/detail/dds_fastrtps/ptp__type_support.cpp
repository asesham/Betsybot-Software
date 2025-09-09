// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from hesai_ros_driver:msg/Ptp.idl
// generated code does not contain a copyright notice
#include "hesai_ros_driver/msg/detail/ptp__rosidl_typesupport_fastrtps_cpp.hpp"
#include "hesai_ros_driver/msg/detail/ptp__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace hesai_ros_driver
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hesai_ros_driver
cdr_serialize(
  const hesai_ros_driver::msg::Ptp & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: ptp_lock_offset
  cdr << ros_message.ptp_lock_offset;
  // Member: ptp_status
  {
    cdr << ros_message.ptp_status;
  }
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hesai_ros_driver
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  hesai_ros_driver::msg::Ptp & ros_message)
{
  // Member: ptp_lock_offset
  cdr >> ros_message.ptp_lock_offset;

  // Member: ptp_status
  {
    cdr >> ros_message.ptp_status;
  }

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hesai_ros_driver
get_serialized_size(
  const hesai_ros_driver::msg::Ptp & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: ptp_lock_offset
  {
    size_t item_size = sizeof(ros_message.ptp_lock_offset);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ptp_status
  {
    size_t array_size = 16;
    size_t item_size = sizeof(ros_message.ptp_status[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_hesai_ros_driver
max_serialized_size_Ptp(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: ptp_lock_offset
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: ptp_status
  {
    size_t array_size = 16;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = hesai_ros_driver::msg::Ptp;
    is_plain =
      (
      offsetof(DataType, ptp_status) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Ptp__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const hesai_ros_driver::msg::Ptp *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Ptp__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<hesai_ros_driver::msg::Ptp *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Ptp__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const hesai_ros_driver::msg::Ptp *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Ptp__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Ptp(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Ptp__callbacks = {
  "hesai_ros_driver::msg",
  "Ptp",
  _Ptp__cdr_serialize,
  _Ptp__cdr_deserialize,
  _Ptp__get_serialized_size,
  _Ptp__max_serialized_size
};

static rosidl_message_type_support_t _Ptp__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Ptp__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace hesai_ros_driver

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_hesai_ros_driver
const rosidl_message_type_support_t *
get_message_type_support_handle<hesai_ros_driver::msg::Ptp>()
{
  return &hesai_ros_driver::msg::typesupport_fastrtps_cpp::_Ptp__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, hesai_ros_driver, msg, Ptp)() {
  return &hesai_ros_driver::msg::typesupport_fastrtps_cpp::_Ptp__handle;
}

#ifdef __cplusplus
}
#endif
