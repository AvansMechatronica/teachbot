// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from teachbot_interfaces:msg/TeachbotPistolState.idl
// generated code does not contain a copyright notice

#ifndef TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_PISTOL_STATE__BUILDER_HPP_
#define TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_PISTOL_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "teachbot_interfaces/msg/detail/teachbot_pistol_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace teachbot_interfaces
{

namespace msg
{

namespace builder
{

class Init_TeachbotPistolState_btn2
{
public:
  explicit Init_TeachbotPistolState_btn2(::teachbot_interfaces::msg::TeachbotPistolState & msg)
  : msg_(msg)
  {}
  ::teachbot_interfaces::msg::TeachbotPistolState btn2(::teachbot_interfaces::msg::TeachbotPistolState::_btn2_type arg)
  {
    msg_.btn2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::teachbot_interfaces::msg::TeachbotPistolState msg_;
};

class Init_TeachbotPistolState_btn1
{
public:
  explicit Init_TeachbotPistolState_btn1(::teachbot_interfaces::msg::TeachbotPistolState & msg)
  : msg_(msg)
  {}
  Init_TeachbotPistolState_btn2 btn1(::teachbot_interfaces::msg::TeachbotPistolState::_btn1_type arg)
  {
    msg_.btn1 = std::move(arg);
    return Init_TeachbotPistolState_btn2(msg_);
  }

private:
  ::teachbot_interfaces::msg::TeachbotPistolState msg_;
};

class Init_TeachbotPistolState_pot_percent
{
public:
  explicit Init_TeachbotPistolState_pot_percent(::teachbot_interfaces::msg::TeachbotPistolState & msg)
  : msg_(msg)
  {}
  Init_TeachbotPistolState_btn1 pot_percent(::teachbot_interfaces::msg::TeachbotPistolState::_pot_percent_type arg)
  {
    msg_.pot_percent = std::move(arg);
    return Init_TeachbotPistolState_btn1(msg_);
  }

private:
  ::teachbot_interfaces::msg::TeachbotPistolState msg_;
};

class Init_TeachbotPistolState_pot_raw
{
public:
  Init_TeachbotPistolState_pot_raw()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TeachbotPistolState_pot_percent pot_raw(::teachbot_interfaces::msg::TeachbotPistolState::_pot_raw_type arg)
  {
    msg_.pot_raw = std::move(arg);
    return Init_TeachbotPistolState_pot_percent(msg_);
  }

private:
  ::teachbot_interfaces::msg::TeachbotPistolState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::teachbot_interfaces::msg::TeachbotPistolState>()
{
  return teachbot_interfaces::msg::builder::Init_TeachbotPistolState_pot_raw();
}

}  // namespace teachbot_interfaces

#endif  // TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_PISTOL_STATE__BUILDER_HPP_
