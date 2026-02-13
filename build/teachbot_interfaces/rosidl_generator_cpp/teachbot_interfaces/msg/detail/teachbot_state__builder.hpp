// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from teachbot_interfaces:msg/TeachbotState.idl
// generated code does not contain a copyright notice

#ifndef TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_STATE__BUILDER_HPP_
#define TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "teachbot_interfaces/msg/detail/teachbot_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace teachbot_interfaces
{

namespace msg
{

namespace builder
{

class Init_TeachbotState_robot_model
{
public:
  explicit Init_TeachbotState_robot_model(::teachbot_interfaces::msg::TeachbotState & msg)
  : msg_(msg)
  {}
  ::teachbot_interfaces::msg::TeachbotState robot_model(::teachbot_interfaces::msg::TeachbotState::_robot_model_type arg)
  {
    msg_.robot_model = std::move(arg);
    return std::move(msg_);
  }

private:
  ::teachbot_interfaces::msg::TeachbotState msg_;
};

class Init_TeachbotState_encoder_frequencies
{
public:
  explicit Init_TeachbotState_encoder_frequencies(::teachbot_interfaces::msg::TeachbotState & msg)
  : msg_(msg)
  {}
  Init_TeachbotState_robot_model encoder_frequencies(::teachbot_interfaces::msg::TeachbotState::_encoder_frequencies_type arg)
  {
    msg_.encoder_frequencies = std::move(arg);
    return Init_TeachbotState_robot_model(msg_);
  }

private:
  ::teachbot_interfaces::msg::TeachbotState msg_;
};

class Init_TeachbotState_encoder_warnings
{
public:
  explicit Init_TeachbotState_encoder_warnings(::teachbot_interfaces::msg::TeachbotState & msg)
  : msg_(msg)
  {}
  Init_TeachbotState_encoder_frequencies encoder_warnings(::teachbot_interfaces::msg::TeachbotState::_encoder_warnings_type arg)
  {
    msg_.encoder_warnings = std::move(arg);
    return Init_TeachbotState_encoder_frequencies(msg_);
  }

private:
  ::teachbot_interfaces::msg::TeachbotState msg_;
};

class Init_TeachbotState_encoder_errors
{
public:
  explicit Init_TeachbotState_encoder_errors(::teachbot_interfaces::msg::TeachbotState & msg)
  : msg_(msg)
  {}
  Init_TeachbotState_encoder_warnings encoder_errors(::teachbot_interfaces::msg::TeachbotState::_encoder_errors_type arg)
  {
    msg_.encoder_errors = std::move(arg);
    return Init_TeachbotState_encoder_warnings(msg_);
  }

private:
  ::teachbot_interfaces::msg::TeachbotState msg_;
};

class Init_TeachbotState_pistol
{
public:
  explicit Init_TeachbotState_pistol(::teachbot_interfaces::msg::TeachbotState & msg)
  : msg_(msg)
  {}
  Init_TeachbotState_encoder_errors pistol(::teachbot_interfaces::msg::TeachbotState::_pistol_type arg)
  {
    msg_.pistol = std::move(arg);
    return Init_TeachbotState_encoder_errors(msg_);
  }

private:
  ::teachbot_interfaces::msg::TeachbotState msg_;
};

class Init_TeachbotState_tcp_rz
{
public:
  explicit Init_TeachbotState_tcp_rz(::teachbot_interfaces::msg::TeachbotState & msg)
  : msg_(msg)
  {}
  Init_TeachbotState_pistol tcp_rz(::teachbot_interfaces::msg::TeachbotState::_tcp_rz_type arg)
  {
    msg_.tcp_rz = std::move(arg);
    return Init_TeachbotState_pistol(msg_);
  }

private:
  ::teachbot_interfaces::msg::TeachbotState msg_;
};

class Init_TeachbotState_tcp_ry
{
public:
  explicit Init_TeachbotState_tcp_ry(::teachbot_interfaces::msg::TeachbotState & msg)
  : msg_(msg)
  {}
  Init_TeachbotState_tcp_rz tcp_ry(::teachbot_interfaces::msg::TeachbotState::_tcp_ry_type arg)
  {
    msg_.tcp_ry = std::move(arg);
    return Init_TeachbotState_tcp_rz(msg_);
  }

private:
  ::teachbot_interfaces::msg::TeachbotState msg_;
};

class Init_TeachbotState_tcp_rx
{
public:
  explicit Init_TeachbotState_tcp_rx(::teachbot_interfaces::msg::TeachbotState & msg)
  : msg_(msg)
  {}
  Init_TeachbotState_tcp_ry tcp_rx(::teachbot_interfaces::msg::TeachbotState::_tcp_rx_type arg)
  {
    msg_.tcp_rx = std::move(arg);
    return Init_TeachbotState_tcp_ry(msg_);
  }

private:
  ::teachbot_interfaces::msg::TeachbotState msg_;
};

class Init_TeachbotState_tcp_z
{
public:
  explicit Init_TeachbotState_tcp_z(::teachbot_interfaces::msg::TeachbotState & msg)
  : msg_(msg)
  {}
  Init_TeachbotState_tcp_rx tcp_z(::teachbot_interfaces::msg::TeachbotState::_tcp_z_type arg)
  {
    msg_.tcp_z = std::move(arg);
    return Init_TeachbotState_tcp_rx(msg_);
  }

private:
  ::teachbot_interfaces::msg::TeachbotState msg_;
};

class Init_TeachbotState_tcp_y
{
public:
  explicit Init_TeachbotState_tcp_y(::teachbot_interfaces::msg::TeachbotState & msg)
  : msg_(msg)
  {}
  Init_TeachbotState_tcp_z tcp_y(::teachbot_interfaces::msg::TeachbotState::_tcp_y_type arg)
  {
    msg_.tcp_y = std::move(arg);
    return Init_TeachbotState_tcp_z(msg_);
  }

private:
  ::teachbot_interfaces::msg::TeachbotState msg_;
};

class Init_TeachbotState_tcp_x
{
public:
  explicit Init_TeachbotState_tcp_x(::teachbot_interfaces::msg::TeachbotState & msg)
  : msg_(msg)
  {}
  Init_TeachbotState_tcp_y tcp_x(::teachbot_interfaces::msg::TeachbotState::_tcp_x_type arg)
  {
    msg_.tcp_x = std::move(arg);
    return Init_TeachbotState_tcp_y(msg_);
  }

private:
  ::teachbot_interfaces::msg::TeachbotState msg_;
};

class Init_TeachbotState_joint_angles_deg
{
public:
  explicit Init_TeachbotState_joint_angles_deg(::teachbot_interfaces::msg::TeachbotState & msg)
  : msg_(msg)
  {}
  Init_TeachbotState_tcp_x joint_angles_deg(::teachbot_interfaces::msg::TeachbotState::_joint_angles_deg_type arg)
  {
    msg_.joint_angles_deg = std::move(arg);
    return Init_TeachbotState_tcp_x(msg_);
  }

private:
  ::teachbot_interfaces::msg::TeachbotState msg_;
};

class Init_TeachbotState_header
{
public:
  Init_TeachbotState_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TeachbotState_joint_angles_deg header(::teachbot_interfaces::msg::TeachbotState::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TeachbotState_joint_angles_deg(msg_);
  }

private:
  ::teachbot_interfaces::msg::TeachbotState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::teachbot_interfaces::msg::TeachbotState>()
{
  return teachbot_interfaces::msg::builder::Init_TeachbotState_header();
}

}  // namespace teachbot_interfaces

#endif  // TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_STATE__BUILDER_HPP_
