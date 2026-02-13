// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from teachbot_interfaces:msg/TeachbotPistolState.idl
// generated code does not contain a copyright notice
#include "teachbot_interfaces/msg/detail/teachbot_pistol_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "teachbot_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "teachbot_interfaces/msg/detail/teachbot_pistol_state__struct.h"
#include "teachbot_interfaces/msg/detail/teachbot_pistol_state__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _TeachbotPistolState__ros_msg_type = teachbot_interfaces__msg__TeachbotPistolState;

static bool _TeachbotPistolState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _TeachbotPistolState__ros_msg_type * ros_message = static_cast<const _TeachbotPistolState__ros_msg_type *>(untyped_ros_message);
  // Field name: pot_raw
  {
    cdr << ros_message->pot_raw;
  }

  // Field name: pot_percent
  {
    cdr << ros_message->pot_percent;
  }

  // Field name: btn1
  {
    cdr << (ros_message->btn1 ? true : false);
  }

  // Field name: btn2
  {
    cdr << (ros_message->btn2 ? true : false);
  }

  return true;
}

static bool _TeachbotPistolState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _TeachbotPistolState__ros_msg_type * ros_message = static_cast<_TeachbotPistolState__ros_msg_type *>(untyped_ros_message);
  // Field name: pot_raw
  {
    cdr >> ros_message->pot_raw;
  }

  // Field name: pot_percent
  {
    cdr >> ros_message->pot_percent;
  }

  // Field name: btn1
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->btn1 = tmp ? true : false;
  }

  // Field name: btn2
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->btn2 = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_teachbot_interfaces
size_t get_serialized_size_teachbot_interfaces__msg__TeachbotPistolState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TeachbotPistolState__ros_msg_type * ros_message = static_cast<const _TeachbotPistolState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name pot_raw
  {
    size_t item_size = sizeof(ros_message->pot_raw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pot_percent
  {
    size_t item_size = sizeof(ros_message->pot_percent);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name btn1
  {
    size_t item_size = sizeof(ros_message->btn1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name btn2
  {
    size_t item_size = sizeof(ros_message->btn2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _TeachbotPistolState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_teachbot_interfaces__msg__TeachbotPistolState(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_teachbot_interfaces
size_t max_serialized_size_teachbot_interfaces__msg__TeachbotPistolState(
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

  // member: pot_raw
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: pot_percent
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: btn1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: btn2
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
    using DataType = teachbot_interfaces__msg__TeachbotPistolState;
    is_plain =
      (
      offsetof(DataType, btn2) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _TeachbotPistolState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_teachbot_interfaces__msg__TeachbotPistolState(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_TeachbotPistolState = {
  "teachbot_interfaces::msg",
  "TeachbotPistolState",
  _TeachbotPistolState__cdr_serialize,
  _TeachbotPistolState__cdr_deserialize,
  _TeachbotPistolState__get_serialized_size,
  _TeachbotPistolState__max_serialized_size
};

static rosidl_message_type_support_t _TeachbotPistolState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_TeachbotPistolState,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, teachbot_interfaces, msg, TeachbotPistolState)() {
  return &_TeachbotPistolState__type_support;
}

#if defined(__cplusplus)
}
#endif
