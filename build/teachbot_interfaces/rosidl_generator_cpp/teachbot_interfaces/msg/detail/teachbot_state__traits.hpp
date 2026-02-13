// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from teachbot_interfaces:msg/TeachbotState.idl
// generated code does not contain a copyright notice

#ifndef TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_STATE__TRAITS_HPP_
#define TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "teachbot_interfaces/msg/detail/teachbot_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pistol'
#include "teachbot_interfaces/msg/detail/teachbot_pistol_state__traits.hpp"

namespace teachbot_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TeachbotState & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: joint_angles_deg
  {
    if (msg.joint_angles_deg.size() == 0) {
      out << "joint_angles_deg: []";
    } else {
      out << "joint_angles_deg: [";
      size_t pending_items = msg.joint_angles_deg.size();
      for (auto item : msg.joint_angles_deg) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: tcp_x
  {
    out << "tcp_x: ";
    rosidl_generator_traits::value_to_yaml(msg.tcp_x, out);
    out << ", ";
  }

  // member: tcp_y
  {
    out << "tcp_y: ";
    rosidl_generator_traits::value_to_yaml(msg.tcp_y, out);
    out << ", ";
  }

  // member: tcp_z
  {
    out << "tcp_z: ";
    rosidl_generator_traits::value_to_yaml(msg.tcp_z, out);
    out << ", ";
  }

  // member: tcp_rx
  {
    out << "tcp_rx: ";
    rosidl_generator_traits::value_to_yaml(msg.tcp_rx, out);
    out << ", ";
  }

  // member: tcp_ry
  {
    out << "tcp_ry: ";
    rosidl_generator_traits::value_to_yaml(msg.tcp_ry, out);
    out << ", ";
  }

  // member: tcp_rz
  {
    out << "tcp_rz: ";
    rosidl_generator_traits::value_to_yaml(msg.tcp_rz, out);
    out << ", ";
  }

  // member: pistol
  {
    out << "pistol: ";
    to_flow_style_yaml(msg.pistol, out);
    out << ", ";
  }

  // member: encoder_errors
  {
    if (msg.encoder_errors.size() == 0) {
      out << "encoder_errors: []";
    } else {
      out << "encoder_errors: [";
      size_t pending_items = msg.encoder_errors.size();
      for (auto item : msg.encoder_errors) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: encoder_warnings
  {
    if (msg.encoder_warnings.size() == 0) {
      out << "encoder_warnings: []";
    } else {
      out << "encoder_warnings: [";
      size_t pending_items = msg.encoder_warnings.size();
      for (auto item : msg.encoder_warnings) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: encoder_frequencies
  {
    if (msg.encoder_frequencies.size() == 0) {
      out << "encoder_frequencies: []";
    } else {
      out << "encoder_frequencies: [";
      size_t pending_items = msg.encoder_frequencies.size();
      for (auto item : msg.encoder_frequencies) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: robot_model
  {
    out << "robot_model: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_model, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TeachbotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: joint_angles_deg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.joint_angles_deg.size() == 0) {
      out << "joint_angles_deg: []\n";
    } else {
      out << "joint_angles_deg:\n";
      for (auto item : msg.joint_angles_deg) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: tcp_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tcp_x: ";
    rosidl_generator_traits::value_to_yaml(msg.tcp_x, out);
    out << "\n";
  }

  // member: tcp_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tcp_y: ";
    rosidl_generator_traits::value_to_yaml(msg.tcp_y, out);
    out << "\n";
  }

  // member: tcp_z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tcp_z: ";
    rosidl_generator_traits::value_to_yaml(msg.tcp_z, out);
    out << "\n";
  }

  // member: tcp_rx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tcp_rx: ";
    rosidl_generator_traits::value_to_yaml(msg.tcp_rx, out);
    out << "\n";
  }

  // member: tcp_ry
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tcp_ry: ";
    rosidl_generator_traits::value_to_yaml(msg.tcp_ry, out);
    out << "\n";
  }

  // member: tcp_rz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "tcp_rz: ";
    rosidl_generator_traits::value_to_yaml(msg.tcp_rz, out);
    out << "\n";
  }

  // member: pistol
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pistol:\n";
    to_block_style_yaml(msg.pistol, out, indentation + 2);
  }

  // member: encoder_errors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.encoder_errors.size() == 0) {
      out << "encoder_errors: []\n";
    } else {
      out << "encoder_errors:\n";
      for (auto item : msg.encoder_errors) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: encoder_warnings
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.encoder_warnings.size() == 0) {
      out << "encoder_warnings: []\n";
    } else {
      out << "encoder_warnings:\n";
      for (auto item : msg.encoder_warnings) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: encoder_frequencies
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.encoder_frequencies.size() == 0) {
      out << "encoder_frequencies: []\n";
    } else {
      out << "encoder_frequencies:\n";
      for (auto item : msg.encoder_frequencies) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: robot_model
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "robot_model: ";
    rosidl_generator_traits::value_to_yaml(msg.robot_model, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TeachbotState & msg, bool use_flow_style = false)
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
  const teachbot_interfaces::msg::TeachbotState & msg,
  std::ostream & out, size_t indentation = 0)
{
  teachbot_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use teachbot_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const teachbot_interfaces::msg::TeachbotState & msg)
{
  return teachbot_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<teachbot_interfaces::msg::TeachbotState>()
{
  return "teachbot_interfaces::msg::TeachbotState";
}

template<>
inline const char * name<teachbot_interfaces::msg::TeachbotState>()
{
  return "teachbot_interfaces/msg/TeachbotState";
}

template<>
struct has_fixed_size<teachbot_interfaces::msg::TeachbotState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<teachbot_interfaces::msg::TeachbotState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<teachbot_interfaces::msg::TeachbotState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_STATE__TRAITS_HPP_
