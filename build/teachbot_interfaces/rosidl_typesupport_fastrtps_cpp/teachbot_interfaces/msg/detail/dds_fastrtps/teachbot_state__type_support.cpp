// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from teachbot_interfaces:msg/TeachbotState.idl
// generated code does not contain a copyright notice
#include "teachbot_interfaces/msg/detail/teachbot_state__rosidl_typesupport_fastrtps_cpp.hpp"
#include "teachbot_interfaces/msg/detail/teachbot_state__struct.hpp"

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
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs

namespace teachbot_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const teachbot_interfaces::msg::TeachbotPistolState &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  teachbot_interfaces::msg::TeachbotPistolState &);
size_t get_serialized_size(
  const teachbot_interfaces::msg::TeachbotPistolState &,
  size_t current_alignment);
size_t
max_serialized_size_TeachbotPistolState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace teachbot_interfaces


namespace teachbot_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_teachbot_interfaces
cdr_serialize(
  const teachbot_interfaces::msg::TeachbotState & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: joint_angles_deg
  {
    cdr << ros_message.joint_angles_deg;
  }
  // Member: tcp_x
  cdr << ros_message.tcp_x;
  // Member: tcp_y
  cdr << ros_message.tcp_y;
  // Member: tcp_z
  cdr << ros_message.tcp_z;
  // Member: tcp_rx
  cdr << ros_message.tcp_rx;
  // Member: tcp_ry
  cdr << ros_message.tcp_ry;
  // Member: tcp_rz
  cdr << ros_message.tcp_rz;
  // Member: pistol
  teachbot_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.pistol,
    cdr);
  // Member: encoder_errors
  {
    cdr << ros_message.encoder_errors;
  }
  // Member: encoder_warnings
  {
    cdr << ros_message.encoder_warnings;
  }
  // Member: encoder_frequencies
  {
    cdr << ros_message.encoder_frequencies;
  }
  // Member: robot_model
  cdr << ros_message.robot_model;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_teachbot_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  teachbot_interfaces::msg::TeachbotState & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: joint_angles_deg
  {
    cdr >> ros_message.joint_angles_deg;
  }

  // Member: tcp_x
  cdr >> ros_message.tcp_x;

  // Member: tcp_y
  cdr >> ros_message.tcp_y;

  // Member: tcp_z
  cdr >> ros_message.tcp_z;

  // Member: tcp_rx
  cdr >> ros_message.tcp_rx;

  // Member: tcp_ry
  cdr >> ros_message.tcp_ry;

  // Member: tcp_rz
  cdr >> ros_message.tcp_rz;

  // Member: pistol
  teachbot_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.pistol);

  // Member: encoder_errors
  {
    cdr >> ros_message.encoder_errors;
  }

  // Member: encoder_warnings
  {
    cdr >> ros_message.encoder_warnings;
  }

  // Member: encoder_frequencies
  {
    cdr >> ros_message.encoder_frequencies;
  }

  // Member: robot_model
  cdr >> ros_message.robot_model;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_teachbot_interfaces
get_serialized_size(
  const teachbot_interfaces::msg::TeachbotState & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: joint_angles_deg
  {
    size_t array_size = 6;
    size_t item_size = sizeof(ros_message.joint_angles_deg[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: tcp_x
  {
    size_t item_size = sizeof(ros_message.tcp_x);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: tcp_y
  {
    size_t item_size = sizeof(ros_message.tcp_y);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: tcp_z
  {
    size_t item_size = sizeof(ros_message.tcp_z);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: tcp_rx
  {
    size_t item_size = sizeof(ros_message.tcp_rx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: tcp_ry
  {
    size_t item_size = sizeof(ros_message.tcp_ry);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: tcp_rz
  {
    size_t item_size = sizeof(ros_message.tcp_rz);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pistol

  current_alignment +=
    teachbot_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.pistol, current_alignment);
  // Member: encoder_errors
  {
    size_t array_size = 6;
    size_t item_size = sizeof(ros_message.encoder_errors[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: encoder_warnings
  {
    size_t array_size = 6;
    size_t item_size = sizeof(ros_message.encoder_warnings[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: encoder_frequencies
  {
    size_t array_size = 6;
    size_t item_size = sizeof(ros_message.encoder_frequencies[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: robot_model
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.robot_model.size() + 1);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_teachbot_interfaces
max_serialized_size_TeachbotState(
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


  // Member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: joint_angles_deg
  {
    size_t array_size = 6;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: tcp_x
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: tcp_y
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: tcp_z
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: tcp_rx
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: tcp_ry
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: tcp_rz
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: pistol
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        teachbot_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_TeachbotPistolState(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: encoder_errors
  {
    size_t array_size = 6;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: encoder_warnings
  {
    size_t array_size = 6;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: encoder_frequencies
  {
    size_t array_size = 6;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: robot_model
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
    using DataType = teachbot_interfaces::msg::TeachbotState;
    is_plain =
      (
      offsetof(DataType, robot_model) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _TeachbotState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const teachbot_interfaces::msg::TeachbotState *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _TeachbotState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<teachbot_interfaces::msg::TeachbotState *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _TeachbotState__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const teachbot_interfaces::msg::TeachbotState *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _TeachbotState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_TeachbotState(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _TeachbotState__callbacks = {
  "teachbot_interfaces::msg",
  "TeachbotState",
  _TeachbotState__cdr_serialize,
  _TeachbotState__cdr_deserialize,
  _TeachbotState__get_serialized_size,
  _TeachbotState__max_serialized_size
};

static rosidl_message_type_support_t _TeachbotState__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_TeachbotState__callbacks,
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
get_message_type_support_handle<teachbot_interfaces::msg::TeachbotState>()
{
  return &teachbot_interfaces::msg::typesupport_fastrtps_cpp::_TeachbotState__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, teachbot_interfaces, msg, TeachbotState)() {
  return &teachbot_interfaces::msg::typesupport_fastrtps_cpp::_TeachbotState__handle;
}

#ifdef __cplusplus
}
#endif
