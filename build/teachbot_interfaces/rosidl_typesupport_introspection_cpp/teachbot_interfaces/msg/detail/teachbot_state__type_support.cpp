// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from teachbot_interfaces:msg/TeachbotState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "teachbot_interfaces/msg/detail/teachbot_state__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace teachbot_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void TeachbotState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) teachbot_interfaces::msg::TeachbotState(_init);
}

void TeachbotState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<teachbot_interfaces::msg::TeachbotState *>(message_memory);
  typed_message->~TeachbotState();
}

size_t size_function__TeachbotState__joint_angles_deg(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__TeachbotState__joint_angles_deg(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__TeachbotState__joint_angles_deg(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void fetch_function__TeachbotState__joint_angles_deg(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TeachbotState__joint_angles_deg(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TeachbotState__joint_angles_deg(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TeachbotState__joint_angles_deg(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

size_t size_function__TeachbotState__encoder_errors(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__TeachbotState__encoder_errors(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<bool, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__TeachbotState__encoder_errors(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<bool, 6> *>(untyped_member);
  return &member[index];
}

void fetch_function__TeachbotState__encoder_errors(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const bool *>(
    get_const_function__TeachbotState__encoder_errors(untyped_member, index));
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = item;
}

void assign_function__TeachbotState__encoder_errors(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<bool *>(
    get_function__TeachbotState__encoder_errors(untyped_member, index));
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  item = value;
}

size_t size_function__TeachbotState__encoder_warnings(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__TeachbotState__encoder_warnings(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<bool, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__TeachbotState__encoder_warnings(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<bool, 6> *>(untyped_member);
  return &member[index];
}

void fetch_function__TeachbotState__encoder_warnings(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const bool *>(
    get_const_function__TeachbotState__encoder_warnings(untyped_member, index));
  auto & value = *reinterpret_cast<bool *>(untyped_value);
  value = item;
}

void assign_function__TeachbotState__encoder_warnings(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<bool *>(
    get_function__TeachbotState__encoder_warnings(untyped_member, index));
  const auto & value = *reinterpret_cast<const bool *>(untyped_value);
  item = value;
}

size_t size_function__TeachbotState__encoder_frequencies(const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * get_const_function__TeachbotState__encoder_frequencies(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void * get_function__TeachbotState__encoder_frequencies(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 6> *>(untyped_member);
  return &member[index];
}

void fetch_function__TeachbotState__encoder_frequencies(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const double *>(
    get_const_function__TeachbotState__encoder_frequencies(untyped_member, index));
  auto & value = *reinterpret_cast<double *>(untyped_value);
  value = item;
}

void assign_function__TeachbotState__encoder_frequencies(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<double *>(
    get_function__TeachbotState__encoder_frequencies(untyped_member, index));
  const auto & value = *reinterpret_cast<const double *>(untyped_value);
  item = value;
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TeachbotState_message_member_array[13] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces::msg::TeachbotState, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "joint_angles_deg",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces::msg::TeachbotState, joint_angles_deg),  // bytes offset in struct
    nullptr,  // default value
    size_function__TeachbotState__joint_angles_deg,  // size() function pointer
    get_const_function__TeachbotState__joint_angles_deg,  // get_const(index) function pointer
    get_function__TeachbotState__joint_angles_deg,  // get(index) function pointer
    fetch_function__TeachbotState__joint_angles_deg,  // fetch(index, &value) function pointer
    assign_function__TeachbotState__joint_angles_deg,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "tcp_x",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces::msg::TeachbotState, tcp_x),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "tcp_y",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces::msg::TeachbotState, tcp_y),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "tcp_z",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces::msg::TeachbotState, tcp_z),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "tcp_rx",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces::msg::TeachbotState, tcp_rx),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "tcp_ry",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces::msg::TeachbotState, tcp_ry),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "tcp_rz",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces::msg::TeachbotState, tcp_rz),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pistol",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<teachbot_interfaces::msg::TeachbotPistolState>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces::msg::TeachbotState, pistol),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "encoder_errors",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces::msg::TeachbotState, encoder_errors),  // bytes offset in struct
    nullptr,  // default value
    size_function__TeachbotState__encoder_errors,  // size() function pointer
    get_const_function__TeachbotState__encoder_errors,  // get_const(index) function pointer
    get_function__TeachbotState__encoder_errors,  // get(index) function pointer
    fetch_function__TeachbotState__encoder_errors,  // fetch(index, &value) function pointer
    assign_function__TeachbotState__encoder_errors,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "encoder_warnings",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces::msg::TeachbotState, encoder_warnings),  // bytes offset in struct
    nullptr,  // default value
    size_function__TeachbotState__encoder_warnings,  // size() function pointer
    get_const_function__TeachbotState__encoder_warnings,  // get_const(index) function pointer
    get_function__TeachbotState__encoder_warnings,  // get(index) function pointer
    fetch_function__TeachbotState__encoder_warnings,  // fetch(index, &value) function pointer
    assign_function__TeachbotState__encoder_warnings,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "encoder_frequencies",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces::msg::TeachbotState, encoder_frequencies),  // bytes offset in struct
    nullptr,  // default value
    size_function__TeachbotState__encoder_frequencies,  // size() function pointer
    get_const_function__TeachbotState__encoder_frequencies,  // get_const(index) function pointer
    get_function__TeachbotState__encoder_frequencies,  // get(index) function pointer
    fetch_function__TeachbotState__encoder_frequencies,  // fetch(index, &value) function pointer
    assign_function__TeachbotState__encoder_frequencies,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "robot_model",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces::msg::TeachbotState, robot_model),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TeachbotState_message_members = {
  "teachbot_interfaces::msg",  // message namespace
  "TeachbotState",  // message name
  13,  // number of fields
  sizeof(teachbot_interfaces::msg::TeachbotState),
  TeachbotState_message_member_array,  // message members
  TeachbotState_init_function,  // function to initialize message memory (memory has to be allocated)
  TeachbotState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TeachbotState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TeachbotState_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace teachbot_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<teachbot_interfaces::msg::TeachbotState>()
{
  return &::teachbot_interfaces::msg::rosidl_typesupport_introspection_cpp::TeachbotState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, teachbot_interfaces, msg, TeachbotState)() {
  return &::teachbot_interfaces::msg::rosidl_typesupport_introspection_cpp::TeachbotState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
