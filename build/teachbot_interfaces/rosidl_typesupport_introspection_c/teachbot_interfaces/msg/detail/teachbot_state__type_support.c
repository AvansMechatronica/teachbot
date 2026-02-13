// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from teachbot_interfaces:msg/TeachbotState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "teachbot_interfaces/msg/detail/teachbot_state__rosidl_typesupport_introspection_c.h"
#include "teachbot_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "teachbot_interfaces/msg/detail/teachbot_state__functions.h"
#include "teachbot_interfaces/msg/detail/teachbot_state__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `pistol`
#include "teachbot_interfaces/msg/teachbot_pistol_state.h"
// Member `pistol`
#include "teachbot_interfaces/msg/detail/teachbot_pistol_state__rosidl_typesupport_introspection_c.h"
// Member `robot_model`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__TeachbotState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  teachbot_interfaces__msg__TeachbotState__init(message_memory);
}

void teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__TeachbotState_fini_function(void * message_memory)
{
  teachbot_interfaces__msg__TeachbotState__fini(message_memory);
}

size_t teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__size_function__TeachbotState__joint_angles_deg(
  const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__get_const_function__TeachbotState__joint_angles_deg(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__get_function__TeachbotState__joint_angles_deg(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__fetch_function__TeachbotState__joint_angles_deg(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__get_const_function__TeachbotState__joint_angles_deg(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__assign_function__TeachbotState__joint_angles_deg(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__get_function__TeachbotState__joint_angles_deg(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

size_t teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__size_function__TeachbotState__encoder_errors(
  const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__get_const_function__TeachbotState__encoder_errors(
  const void * untyped_member, size_t index)
{
  const bool * member =
    (const bool *)(untyped_member);
  return &member[index];
}

void * teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__get_function__TeachbotState__encoder_errors(
  void * untyped_member, size_t index)
{
  bool * member =
    (bool *)(untyped_member);
  return &member[index];
}

void teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__fetch_function__TeachbotState__encoder_errors(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__get_const_function__TeachbotState__encoder_errors(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__assign_function__TeachbotState__encoder_errors(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__get_function__TeachbotState__encoder_errors(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

size_t teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__size_function__TeachbotState__encoder_warnings(
  const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__get_const_function__TeachbotState__encoder_warnings(
  const void * untyped_member, size_t index)
{
  const bool * member =
    (const bool *)(untyped_member);
  return &member[index];
}

void * teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__get_function__TeachbotState__encoder_warnings(
  void * untyped_member, size_t index)
{
  bool * member =
    (bool *)(untyped_member);
  return &member[index];
}

void teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__fetch_function__TeachbotState__encoder_warnings(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const bool * item =
    ((const bool *)
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__get_const_function__TeachbotState__encoder_warnings(untyped_member, index));
  bool * value =
    (bool *)(untyped_value);
  *value = *item;
}

void teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__assign_function__TeachbotState__encoder_warnings(
  void * untyped_member, size_t index, const void * untyped_value)
{
  bool * item =
    ((bool *)
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__get_function__TeachbotState__encoder_warnings(untyped_member, index));
  const bool * value =
    (const bool *)(untyped_value);
  *item = *value;
}

size_t teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__size_function__TeachbotState__encoder_frequencies(
  const void * untyped_member)
{
  (void)untyped_member;
  return 6;
}

const void * teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__get_const_function__TeachbotState__encoder_frequencies(
  const void * untyped_member, size_t index)
{
  const double * member =
    (const double *)(untyped_member);
  return &member[index];
}

void * teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__get_function__TeachbotState__encoder_frequencies(
  void * untyped_member, size_t index)
{
  double * member =
    (double *)(untyped_member);
  return &member[index];
}

void teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__fetch_function__TeachbotState__encoder_frequencies(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__get_const_function__TeachbotState__encoder_frequencies(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__assign_function__TeachbotState__encoder_frequencies(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__get_function__TeachbotState__encoder_frequencies(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__TeachbotState_message_member_array[13] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces__msg__TeachbotState, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joint_angles_deg",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces__msg__TeachbotState, joint_angles_deg),  // bytes offset in struct
    NULL,  // default value
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__size_function__TeachbotState__joint_angles_deg,  // size() function pointer
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__get_const_function__TeachbotState__joint_angles_deg,  // get_const(index) function pointer
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__get_function__TeachbotState__joint_angles_deg,  // get(index) function pointer
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__fetch_function__TeachbotState__joint_angles_deg,  // fetch(index, &value) function pointer
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__assign_function__TeachbotState__joint_angles_deg,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tcp_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces__msg__TeachbotState, tcp_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tcp_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces__msg__TeachbotState, tcp_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tcp_z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces__msg__TeachbotState, tcp_z),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tcp_rx",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces__msg__TeachbotState, tcp_rx),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tcp_ry",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces__msg__TeachbotState, tcp_ry),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "tcp_rz",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces__msg__TeachbotState, tcp_rz),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pistol",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces__msg__TeachbotState, pistol),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "encoder_errors",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces__msg__TeachbotState, encoder_errors),  // bytes offset in struct
    NULL,  // default value
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__size_function__TeachbotState__encoder_errors,  // size() function pointer
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__get_const_function__TeachbotState__encoder_errors,  // get_const(index) function pointer
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__get_function__TeachbotState__encoder_errors,  // get(index) function pointer
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__fetch_function__TeachbotState__encoder_errors,  // fetch(index, &value) function pointer
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__assign_function__TeachbotState__encoder_errors,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "encoder_warnings",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces__msg__TeachbotState, encoder_warnings),  // bytes offset in struct
    NULL,  // default value
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__size_function__TeachbotState__encoder_warnings,  // size() function pointer
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__get_const_function__TeachbotState__encoder_warnings,  // get_const(index) function pointer
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__get_function__TeachbotState__encoder_warnings,  // get(index) function pointer
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__fetch_function__TeachbotState__encoder_warnings,  // fetch(index, &value) function pointer
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__assign_function__TeachbotState__encoder_warnings,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "encoder_frequencies",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    6,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces__msg__TeachbotState, encoder_frequencies),  // bytes offset in struct
    NULL,  // default value
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__size_function__TeachbotState__encoder_frequencies,  // size() function pointer
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__get_const_function__TeachbotState__encoder_frequencies,  // get_const(index) function pointer
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__get_function__TeachbotState__encoder_frequencies,  // get(index) function pointer
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__fetch_function__TeachbotState__encoder_frequencies,  // fetch(index, &value) function pointer
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__assign_function__TeachbotState__encoder_frequencies,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_model",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces__msg__TeachbotState, robot_model),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__TeachbotState_message_members = {
  "teachbot_interfaces__msg",  // message namespace
  "TeachbotState",  // message name
  13,  // number of fields
  sizeof(teachbot_interfaces__msg__TeachbotState),
  teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__TeachbotState_message_member_array,  // message members
  teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__TeachbotState_init_function,  // function to initialize message memory (memory has to be allocated)
  teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__TeachbotState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__TeachbotState_message_type_support_handle = {
  0,
  &teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__TeachbotState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_teachbot_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teachbot_interfaces, msg, TeachbotState)() {
  teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__TeachbotState_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__TeachbotState_message_member_array[8].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, teachbot_interfaces, msg, TeachbotPistolState)();
  if (!teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__TeachbotState_message_type_support_handle.typesupport_identifier) {
    teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__TeachbotState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &teachbot_interfaces__msg__TeachbotState__rosidl_typesupport_introspection_c__TeachbotState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
