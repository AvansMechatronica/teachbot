// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from teachbot_interfaces:msg/TeachbotPistolState.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "teachbot_interfaces/msg/detail/teachbot_pistol_state__struct.hpp"
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

void TeachbotPistolState_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) teachbot_interfaces::msg::TeachbotPistolState(_init);
}

void TeachbotPistolState_fini_function(void * message_memory)
{
  auto typed_message = static_cast<teachbot_interfaces::msg::TeachbotPistolState *>(message_memory);
  typed_message->~TeachbotPistolState();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember TeachbotPistolState_message_member_array[4] = {
  {
    "pot_raw",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces::msg::TeachbotPistolState, pot_raw),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "pot_percent",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces::msg::TeachbotPistolState, pot_percent),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "btn1",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces::msg::TeachbotPistolState, btn1),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "btn2",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(teachbot_interfaces::msg::TeachbotPistolState, btn2),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers TeachbotPistolState_message_members = {
  "teachbot_interfaces::msg",  // message namespace
  "TeachbotPistolState",  // message name
  4,  // number of fields
  sizeof(teachbot_interfaces::msg::TeachbotPistolState),
  TeachbotPistolState_message_member_array,  // message members
  TeachbotPistolState_init_function,  // function to initialize message memory (memory has to be allocated)
  TeachbotPistolState_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t TeachbotPistolState_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &TeachbotPistolState_message_members,
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
get_message_type_support_handle<teachbot_interfaces::msg::TeachbotPistolState>()
{
  return &::teachbot_interfaces::msg::rosidl_typesupport_introspection_cpp::TeachbotPistolState_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, teachbot_interfaces, msg, TeachbotPistolState)() {
  return &::teachbot_interfaces::msg::rosidl_typesupport_introspection_cpp::TeachbotPistolState_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
