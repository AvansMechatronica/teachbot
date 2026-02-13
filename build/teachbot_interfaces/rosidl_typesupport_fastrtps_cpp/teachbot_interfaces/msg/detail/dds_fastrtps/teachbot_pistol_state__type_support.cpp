// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from teachbot_interfaces:msg/TeachbotPistolState.idl
// generated code does not contain a copyright notice
#include "teachbot_interfaces/msg/detail/teachbot_pistol_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "teachbot_interfaces/msg/detail/teachbot_pistol_state__struct.hpp"

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

namespace teachbot_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_teachbot_interfaces
cdr_serialize(
  const teachbot_interfaces::msg::TeachbotPistolState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: pot_raw
  cdr << ros_message.pot_raw;
  // Member: pot_percent
  cdr << ros_message.pot_percent;
  // Member: btn1
  cdr << (ros_message.btn1 ? true : false);
  // Member: btn2
  cdr << (ros_message.btn2 ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_teachbot_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  teachbot_interfaces::msg::TeachbotPistolState & ros_message)
{
  // Member: pot_raw
  cdr >> ros_message.pot_raw;

  // Member: pot_percent
  cdr >> ros_message.pot_percent;

  // Member: btn1
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.btn1 = tmp ? true : false;
  }

  // Member: btn2
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.btn2 = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_teachbot_interfaces
get_serialized_size(
  const teachbot_interfaces::msg::TeachbotPistolState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: pot_raw
  {
    size_t item_size = sizeof(ros_message.pot_raw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pot_percent
  {
    size_t item_size = sizeof(ros_message.pot_percent);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: btn1
  {
    size_t item_size = sizeof(ros_message.btn1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: btn2
  {
    size_t item_size = sizeof(ros_message.btn2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_teachbot_interfaces
max_serialized_size_TeachbotPistolState(
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


  // Member: pot_raw
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: pot_percent
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: btn1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: btn2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = teachbot_interfaces::msg::TeachbotPistolState;
    is_plain =
      (
      offsetof(DataType, btn2) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _TeachbotPistolState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const teachbot_interfaces::msg::TeachbotPistolState *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _TeachbotPistolState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<teachbot_interfaces::msg::TeachbotPistolState *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _TeachbotPistolState__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const teachbot_interfaces::msg::TeachbotPistolState *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _TeachbotPistolState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_TeachbotPistolState(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _TeachbotPistolState__callbacks = {
  "teachbot_interfaces::msg",
  "TeachbotPistolState",
  _TeachbotPistolState__cdr_serialize,
  _TeachbotPistolState__cdr_deserialize,
  _TeachbotPistolState__get_serialized_size,
  _TeachbotPistolState__max_serialized_size
};

static rosidl_message_type_support_t _TeachbotPistolState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_TeachbotPistolState__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace teachbot_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_teachbot_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<teachbot_interfaces::msg::TeachbotPistolState>()
{
  return &teachbot_interfaces::msg::typesupport_fastrtps_cpp::_TeachbotPistolState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, teachbot_interfaces, msg, TeachbotPistolState)() {
  return &teachbot_interfaces::msg::typesupport_fastrtps_cpp::_TeachbotPistolState__handle;
}

#ifdef __cplusplus
}
#endif
