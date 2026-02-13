// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from teachbot_interfaces:msg/TeachbotState.idl
// generated code does not contain a copyright notice

#ifndef TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_STATE__STRUCT_H_
#define TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'pistol'
#include "teachbot_interfaces/msg/detail/teachbot_pistol_state__struct.h"
// Member 'robot_model'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/TeachbotState in the package teachbot_interfaces.
/**
  * TeachbotState.msg
  * Complete state of the TOS Teachbot including joints, TCP, and controls
 */
typedef struct teachbot_interfaces__msg__TeachbotState
{
  /// Header with timestamp
  std_msgs__msg__Header header;
  /// Joint angles in degrees (6 DOF)
  double joint_angles_deg[6];
  /// TCP position from forward kinematics (mm)
  double tcp_x;
  double tcp_y;
  double tcp_z;
  /// TCP orientation (degrees)
  double tcp_rx;
  double tcp_ry;
  double tcp_rz;
  /// Gripper/control inputs
  teachbot_interfaces__msg__TeachbotPistolState pistol;
  /// Encoder status per joint
  bool encoder_errors[6];
  bool encoder_warnings[6];
  double encoder_frequencies[6];
  /// Model information
  rosidl_runtime_c__String robot_model;
} teachbot_interfaces__msg__TeachbotState;

// Struct for a sequence of teachbot_interfaces__msg__TeachbotState.
typedef struct teachbot_interfaces__msg__TeachbotState__Sequence
{
  teachbot_interfaces__msg__TeachbotState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} teachbot_interfaces__msg__TeachbotState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_STATE__STRUCT_H_
