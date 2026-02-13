// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from teachbot_interfaces:msg/TeachbotPistolState.idl
// generated code does not contain a copyright notice

#ifndef TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_PISTOL_STATE__TRAITS_HPP_
#define TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_PISTOL_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "teachbot_interfaces/msg/detail/teachbot_pistol_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace teachbot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TeachbotPistolState & msg,
  std::ostream & out)
{
  out << "{";
  // member: pot_raw
  {
    out << "pot_raw: ";
    rosidl_generator_traits::value_to_yaml(msg.pot_raw, out);
    out << ", ";
  }

  // member: pot_percent
  {
    out << "pot_percent: ";
    rosidl_generator_traits::value_to_yaml(msg.pot_percent, out);
    out << ", ";
  }

  // member: btn1
  {
    out << "btn1: ";
    rosidl_generator_traits::value_to_yaml(msg.btn1, out);
    out << ", ";
  }

  // member: btn2
  {
    out << "btn2: ";
    rosidl_generator_traits::value_to_yaml(msg.btn2, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TeachbotPistolState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pot_raw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pot_raw: ";
    rosidl_generator_traits::value_to_yaml(msg.pot_raw, out);
    out << "\n";
  }

  // member: pot_percent
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pot_percent: ";
    rosidl_generator_traits::value_to_yaml(msg.pot_percent, out);
    out << "\n";
  }

  // member: btn1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "btn1: ";
    rosidl_generator_traits::value_to_yaml(msg.btn1, out);
    out << "\n";
  }

  // member: btn2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "btn2: ";
    rosidl_generator_traits::value_to_yaml(msg.btn2, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TeachbotPistolState & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace teachbot_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use teachbot_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const teachbot_interfaces::msg::TeachbotPistolState & msg,
  std::ostream & out, size_t indentation = 0)
{
  teachbot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use teachbot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const teachbot_interfaces::msg::TeachbotPistolState & msg)
{
  return teachbot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<teachbot_interfaces::msg::TeachbotPistolState>()
{
  return "teachbot_interfaces::msg::TeachbotPistolState";
}

template<>
inline const char * name<teachbot_interfaces::msg::TeachbotPistolState>()
{
  return "teachbot_interfaces/msg/TeachbotPistolState";
}

template<>
struct has_fixed_size<teachbot_interfaces::msg::TeachbotPistolState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<teachbot_interfaces::msg::TeachbotPistolState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<teachbot_interfaces::msg::TeachbotPistolState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_PISTOL_STATE__TRAITS_HPP_
