// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from teachbot_interfaces:msg/TeachbotPistolState.idl
// generated code does not contain a copyright notice

#ifndef TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_PISTOL_STATE__STRUCT_H_
#define TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_PISTOL_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/TeachbotPistolState in the package teachbot_interfaces.
/**
  * TeachbotPistol.msg
 */
typedef struct teachbot_interfaces__msg__TeachbotPistolState
{
  /// Gripper/control inputs
  /// Raw potentiometer value (0-1023)
  int32_t pot_raw;
  /// Mapped potentiometer percentage (0-100)
  int32_t pot_percent;
  /// Button 1 state
  bool btn1;
  /// Button 2 state
  bool btn2;
} teachbot_interfaces__msg__TeachbotPistolState;

// Struct for a sequence of teachbot_interfaces__msg__TeachbotPistolState.
typedef struct teachbot_interfaces__msg__TeachbotPistolState__Sequence
{
  teachbot_interfaces__msg__TeachbotPistolState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} teachbot_interfaces__msg__TeachbotPistolState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_PISTOL_STATE__STRUCT_H_
