// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from teachbot_interfaces:msg/TeachbotState.idl
// generated code does not contain a copyright notice
#include "teachbot_interfaces/msg/detail/teachbot_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "teachbot_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "teachbot_interfaces/msg/detail/teachbot_state__struct.h"
#include "teachbot_interfaces/msg/detail/teachbot_state__functions.h"
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

#include "rosidl_runtime_c/string.h"  // robot_model
#include "rosidl_runtime_c/string_functions.h"  // robot_model
#include "std_msgs/msg/detail/header__functions.h"  // header
#include "teachbot_interfaces/msg/detail/teachbot_pistol_state__functions.h"  // pistol

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_teachbot_interfaces
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_teachbot_interfaces
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_teachbot_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();
size_t get_serialized_size_teachbot_interfaces__msg__TeachbotPistolState(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_teachbot_interfaces__msg__TeachbotPistolState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, teachbot_interfaces, msg, TeachbotPistolState)();


using _TeachbotState__ros_msg_type = teachbot_interfaces__msg__TeachbotState;

static bool _TeachbotState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _TeachbotState__ros_msg_type * ros_message = static_cast<const _TeachbotState__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: joint_angles_deg
  {
    size_t size = 6;
    auto array_ptr = ros_message->joint_angles_deg;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: tcp_x
  {
    cdr << ros_message->tcp_x;
  }

  // Field name: tcp_y
  {
    cdr << ros_message->tcp_y;
  }

  // Field name: tcp_z
  {
    cdr << ros_message->tcp_z;
  }

  // Field name: tcp_rx
  {
    cdr << ros_message->tcp_rx;
  }

  // Field name: tcp_ry
  {
    cdr << ros_message->tcp_ry;
  }

  // Field name: tcp_rz
  {
    cdr << ros_message->tcp_rz;
  }

  // Field name: pistol
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, teachbot_interfaces, msg, TeachbotPistolState
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->pistol, cdr))
    {
      return false;
    }
  }

  // Field name: encoder_errors
  {
    size_t size = 6;
    auto array_ptr = ros_message->encoder_errors;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: encoder_warnings
  {
    size_t size = 6;
    auto array_ptr = ros_message->encoder_warnings;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: encoder_frequencies
  {
    size_t size = 6;
    auto array_ptr = ros_message->encoder_frequencies;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: robot_model
  {
    const rosidl_runtime_c__String * str = &ros_message->robot_model;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  return true;
}

static bool _TeachbotState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _TeachbotState__ros_msg_type * ros_message = static_cast<_TeachbotState__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: joint_angles_deg
  {
    size_t size = 6;
    auto array_ptr = ros_message->joint_angles_deg;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: tcp_x
  {
    cdr >> ros_message->tcp_x;
  }

  // Field name: tcp_y
  {
    cdr >> ros_message->tcp_y;
  }

  // Field name: tcp_z
  {
    cdr >> ros_message->tcp_z;
  }

  // Field name: tcp_rx
  {
    cdr >> ros_message->tcp_rx;
  }

  // Field name: tcp_ry
  {
    cdr >> ros_message->tcp_ry;
  }

  // Field name: tcp_rz
  {
    cdr >> ros_message->tcp_rz;
  }

  // Field name: pistol
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, teachbot_interfaces, msg, TeachbotPistolState
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->pistol))
    {
      return false;
    }
  }

  // Field name: encoder_errors
  {
    size_t size = 6;
    auto array_ptr = ros_message->encoder_errors;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  // Field name: encoder_warnings
  {
    size_t size = 6;
    auto array_ptr = ros_message->encoder_warnings;
    for (size_t i = 0; i < size; ++i) {
      uint8_t tmp;
      cdr >> tmp;
      array_ptr[i] = tmp ? true : false;
    }
  }

  // Field name: encoder_frequencies
  {
    size_t size = 6;
    auto array_ptr = ros_message->encoder_frequencies;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: robot_model
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->robot_model.data) {
      rosidl_runtime_c__String__init(&ros_message->robot_model);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->robot_model,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'robot_model'\n");
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_teachbot_interfaces
size_t get_serialized_size_teachbot_interfaces__msg__TeachbotState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TeachbotState__ros_msg_type * ros_message = static_cast<const _TeachbotState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name joint_angles_deg
  {
    size_t array_size = 6;
    auto array_ptr = ros_message->joint_angles_deg;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name tcp_x
  {
    size_t item_size = sizeof(ros_message->tcp_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name tcp_y
  {
    size_t item_size = sizeof(ros_message->tcp_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name tcp_z
  {
    size_t item_size = sizeof(ros_message->tcp_z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name tcp_rx
  {
    size_t item_size = sizeof(ros_message->tcp_rx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name tcp_ry
  {
    size_t item_size = sizeof(ros_message->tcp_ry);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name tcp_rz
  {
    size_t item_size = sizeof(ros_message->tcp_rz);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pistol

  current_alignment += get_serialized_size_teachbot_interfaces__msg__TeachbotPistolState(
    &(ros_message->pistol), current_alignment);
  // field.name encoder_errors
  {
    size_t array_size = 6;
    auto array_ptr = ros_message->encoder_errors;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name encoder_warnings
  {
    size_t array_size = 6;
    auto array_ptr = ros_message->encoder_warnings;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name encoder_frequencies
  {
    size_t array_size = 6;
    auto array_ptr = ros_message->encoder_frequencies;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name robot_model
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->robot_model.size + 1);

  return current_alignment - initial_alignment;
}

static uint32_t _TeachbotState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_teachbot_interfaces__msg__TeachbotState(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_teachbot_interfaces
size_t max_serialized_size_teachbot_interfaces__msg__TeachbotState(
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

  // member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: joint_angles_deg
  {
    size_t array_size = 6;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: tcp_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: tcp_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: tcp_z
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: tcp_rx
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: tcp_ry
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: tcp_rz
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: pistol
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_teachbot_interfaces__msg__TeachbotPistolState(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: encoder_errors
  {
    size_t array_size = 6;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: encoder_warnings
  {
    size_t array_size = 6;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: encoder_frequencies
  {
    size_t array_size = 6;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: robot_model
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = teachbot_interfaces__msg__TeachbotState;
    is_plain =
      (
      offsetof(DataType, robot_model) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _TeachbotState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_teachbot_interfaces__msg__TeachbotState(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_TeachbotState = {
  "teachbot_interfaces::msg",
  "TeachbotState",
  _TeachbotState__cdr_serialize,
  _TeachbotState__cdr_deserialize,
  _TeachbotState__get_serialized_size,
  _TeachbotState__max_serialized_size
};

static rosidl_message_type_support_t _TeachbotState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_TeachbotState,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, teachbot_interfaces, msg, TeachbotState)() {
  return &_TeachbotState__type_support;
}

#if defined(__cplusplus)
}
#endif
