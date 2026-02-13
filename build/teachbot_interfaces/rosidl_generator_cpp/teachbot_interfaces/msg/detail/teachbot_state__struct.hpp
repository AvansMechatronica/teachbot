// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from teachbot_interfaces:msg/TeachbotState.idl
// generated code does not contain a copyright notice

#ifndef TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_STATE__STRUCT_HPP_
#define TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'pistol'
#include "teachbot_interfaces/msg/detail/teachbot_pistol_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__teachbot_interfaces__msg__TeachbotState __attribute__((deprecated))
#else
# define DEPRECATED__teachbot_interfaces__msg__TeachbotState __declspec(deprecated)
#endif

namespace teachbot_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TeachbotState_
{
  using Type = TeachbotState_<ContainerAllocator>;

  explicit TeachbotState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pistol(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 6>::iterator, double>(this->joint_angles_deg.begin(), this->joint_angles_deg.end(), 0.0);
      this->tcp_x = 0.0;
      this->tcp_y = 0.0;
      this->tcp_z = 0.0;
      this->tcp_rx = 0.0;
      this->tcp_ry = 0.0;
      this->tcp_rz = 0.0;
      std::fill<typename std::array<bool, 6>::iterator, bool>(this->encoder_errors.begin(), this->encoder_errors.end(), false);
      std::fill<typename std::array<bool, 6>::iterator, bool>(this->encoder_warnings.begin(), this->encoder_warnings.end(), false);
      std::fill<typename std::array<double, 6>::iterator, double>(this->encoder_frequencies.begin(), this->encoder_frequencies.end(), 0.0);
      this->robot_model = "";
    }
  }

  explicit TeachbotState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    joint_angles_deg(_alloc),
    pistol(_alloc, _init),
    encoder_errors(_alloc),
    encoder_warnings(_alloc),
    encoder_frequencies(_alloc),
    robot_model(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      std::fill<typename std::array<double, 6>::iterator, double>(this->joint_angles_deg.begin(), this->joint_angles_deg.end(), 0.0);
      this->tcp_x = 0.0;
      this->tcp_y = 0.0;
      this->tcp_z = 0.0;
      this->tcp_rx = 0.0;
      this->tcp_ry = 0.0;
      this->tcp_rz = 0.0;
      std::fill<typename std::array<bool, 6>::iterator, bool>(this->encoder_errors.begin(), this->encoder_errors.end(), false);
      std::fill<typename std::array<bool, 6>::iterator, bool>(this->encoder_warnings.begin(), this->encoder_warnings.end(), false);
      std::fill<typename std::array<double, 6>::iterator, double>(this->encoder_frequencies.begin(), this->encoder_frequencies.end(), 0.0);
      this->robot_model = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _joint_angles_deg_type =
    std::array<double, 6>;
  _joint_angles_deg_type joint_angles_deg;
  using _tcp_x_type =
    double;
  _tcp_x_type tcp_x;
  using _tcp_y_type =
    double;
  _tcp_y_type tcp_y;
  using _tcp_z_type =
    double;
  _tcp_z_type tcp_z;
  using _tcp_rx_type =
    double;
  _tcp_rx_type tcp_rx;
  using _tcp_ry_type =
    double;
  _tcp_ry_type tcp_ry;
  using _tcp_rz_type =
    double;
  _tcp_rz_type tcp_rz;
  using _pistol_type =
    teachbot_interfaces::msg::TeachbotPistolState_<ContainerAllocator>;
  _pistol_type pistol;
  using _encoder_errors_type =
    std::array<bool, 6>;
  _encoder_errors_type encoder_errors;
  using _encoder_warnings_type =
    std::array<bool, 6>;
  _encoder_warnings_type encoder_warnings;
  using _encoder_frequencies_type =
    std::array<double, 6>;
  _encoder_frequencies_type encoder_frequencies;
  using _robot_model_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _robot_model_type robot_model;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__joint_angles_deg(
    const std::array<double, 6> & _arg)
  {
    this->joint_angles_deg = _arg;
    return *this;
  }
  Type & set__tcp_x(
    const double & _arg)
  {
    this->tcp_x = _arg;
    return *this;
  }
  Type & set__tcp_y(
    const double & _arg)
  {
    this->tcp_y = _arg;
    return *this;
  }
  Type & set__tcp_z(
    const double & _arg)
  {
    this->tcp_z = _arg;
    return *this;
  }
  Type & set__tcp_rx(
    const double & _arg)
  {
    this->tcp_rx = _arg;
    return *this;
  }
  Type & set__tcp_ry(
    const double & _arg)
  {
    this->tcp_ry = _arg;
    return *this;
  }
  Type & set__tcp_rz(
    const double & _arg)
  {
    this->tcp_rz = _arg;
    return *this;
  }
  Type & set__pistol(
    const teachbot_interfaces::msg::TeachbotPistolState_<ContainerAllocator> & _arg)
  {
    this->pistol = _arg;
    return *this;
  }
  Type & set__encoder_errors(
    const std::array<bool, 6> & _arg)
  {
    this->encoder_errors = _arg;
    return *this;
  }
  Type & set__encoder_warnings(
    const std::array<bool, 6> & _arg)
  {
    this->encoder_warnings = _arg;
    return *this;
  }
  Type & set__encoder_frequencies(
    const std::array<double, 6> & _arg)
  {
    this->encoder_frequencies = _arg;
    return *this;
  }
  Type & set__robot_model(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->robot_model = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    teachbot_interfaces::msg::TeachbotState_<ContainerAllocator> *;
  using ConstRawPtr =
    const teachbot_interfaces::msg::TeachbotState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<teachbot_interfaces::msg::TeachbotState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<teachbot_interfaces::msg::TeachbotState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      teachbot_interfaces::msg::TeachbotState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<teachbot_interfaces::msg::TeachbotState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      teachbot_interfaces::msg::TeachbotState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<teachbot_interfaces::msg::TeachbotState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<teachbot_interfaces::msg::TeachbotState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<teachbot_interfaces::msg::TeachbotState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__teachbot_interfaces__msg__TeachbotState
    std::shared_ptr<teachbot_interfaces::msg::TeachbotState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__teachbot_interfaces__msg__TeachbotState
    std::shared_ptr<teachbot_interfaces::msg::TeachbotState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TeachbotState_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->joint_angles_deg != other.joint_angles_deg) {
      return false;
    }
    if (this->tcp_x != other.tcp_x) {
      return false;
    }
    if (this->tcp_y != other.tcp_y) {
      return false;
    }
    if (this->tcp_z != other.tcp_z) {
      return false;
    }
    if (this->tcp_rx != other.tcp_rx) {
      return false;
    }
    if (this->tcp_ry != other.tcp_ry) {
      return false;
    }
    if (this->tcp_rz != other.tcp_rz) {
      return false;
    }
    if (this->pistol != other.pistol) {
      return false;
    }
    if (this->encoder_errors != other.encoder_errors) {
      return false;
    }
    if (this->encoder_warnings != other.encoder_warnings) {
      return false;
    }
    if (this->encoder_frequencies != other.encoder_frequencies) {
      return false;
    }
    if (this->robot_model != other.robot_model) {
      return false;
    }
    return true;
  }
  bool operator!=(const TeachbotState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TeachbotState_

// alias to use template instance with default allocator
using TeachbotState =
  teachbot_interfaces::msg::TeachbotState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace teachbot_interfaces

#endif  // TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_STATE__STRUCT_HPP_
