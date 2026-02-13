// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from teachbot_interfaces:msg/TeachbotPistolState.idl
// generated code does not contain a copyright notice

#ifndef TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_PISTOL_STATE__STRUCT_HPP_
#define TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_PISTOL_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__teachbot_interfaces__msg__TeachbotPistolState __attribute__((deprecated))
#else
# define DEPRECATED__teachbot_interfaces__msg__TeachbotPistolState __declspec(deprecated)
#endif

namespace teachbot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TeachbotPistolState_
{
  using Type = TeachbotPistolState_<ContainerAllocator>;

  explicit TeachbotPistolState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pot_raw = 0l;
      this->pot_percent = 0l;
      this->btn1 = false;
      this->btn2 = false;
    }
  }

  explicit TeachbotPistolState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pot_raw = 0l;
      this->pot_percent = 0l;
      this->btn1 = false;
      this->btn2 = false;
    }
  }

  // field types and members
  using _pot_raw_type =
    int32_t;
  _pot_raw_type pot_raw;
  using _pot_percent_type =
    int32_t;
  _pot_percent_type pot_percent;
  using _btn1_type =
    bool;
  _btn1_type btn1;
  using _btn2_type =
    bool;
  _btn2_type btn2;

  // setters for named parameter idiom
  Type & set__pot_raw(
    const int32_t & _arg)
  {
    this->pot_raw = _arg;
    return *this;
  }
  Type & set__pot_percent(
    const int32_t & _arg)
  {
    this->pot_percent = _arg;
    return *this;
  }
  Type & set__btn1(
    const bool & _arg)
  {
    this->btn1 = _arg;
    return *this;
  }
  Type & set__btn2(
    const bool & _arg)
  {
    this->btn2 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    teachbot_interfaces::msg::TeachbotPistolState_<ContainerAllocator> *;
  using ConstRawPtr =
    const teachbot_interfaces::msg::TeachbotPistolState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<teachbot_interfaces::msg::TeachbotPistolState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<teachbot_interfaces::msg::TeachbotPistolState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      teachbot_interfaces::msg::TeachbotPistolState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<teachbot_interfaces::msg::TeachbotPistolState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      teachbot_interfaces::msg::TeachbotPistolState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<teachbot_interfaces::msg::TeachbotPistolState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<teachbot_interfaces::msg::TeachbotPistolState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<teachbot_interfaces::msg::TeachbotPistolState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__teachbot_interfaces__msg__TeachbotPistolState
    std::shared_ptr<teachbot_interfaces::msg::TeachbotPistolState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__teachbot_interfaces__msg__TeachbotPistolState
    std::shared_ptr<teachbot_interfaces::msg::TeachbotPistolState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TeachbotPistolState_ & other) const
  {
    if (this->pot_raw != other.pot_raw) {
      return false;
    }
    if (this->pot_percent != other.pot_percent) {
      return false;
    }
    if (this->btn1 != other.btn1) {
      return false;
    }
    if (this->btn2 != other.btn2) {
      return false;
    }
    return true;
  }
  bool operator!=(const TeachbotPistolState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TeachbotPistolState_

// alias to use template instance with default allocator
using TeachbotPistolState =
  teachbot_interfaces::msg::TeachbotPistolState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace teachbot_interfaces

#endif  // TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_PISTOL_STATE__STRUCT_HPP_
