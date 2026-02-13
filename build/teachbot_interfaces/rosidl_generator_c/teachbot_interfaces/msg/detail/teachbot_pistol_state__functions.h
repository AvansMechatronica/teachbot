// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from teachbot_interfaces:msg/TeachbotPistolState.idl
// generated code does not contain a copyright notice

#ifndef TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_PISTOL_STATE__FUNCTIONS_H_
#define TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_PISTOL_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "teachbot_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "teachbot_interfaces/msg/detail/teachbot_pistol_state__struct.h"

/// Initialize msg/TeachbotPistolState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * teachbot_interfaces__msg__TeachbotPistolState
 * )) before or use
 * teachbot_interfaces__msg__TeachbotPistolState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_teachbot_interfaces
bool
teachbot_interfaces__msg__TeachbotPistolState__init(teachbot_interfaces__msg__TeachbotPistolState * msg);

/// Finalize msg/TeachbotPistolState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_teachbot_interfaces
void
teachbot_interfaces__msg__TeachbotPistolState__fini(teachbot_interfaces__msg__TeachbotPistolState * msg);

/// Create msg/TeachbotPistolState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * teachbot_interfaces__msg__TeachbotPistolState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_teachbot_interfaces
teachbot_interfaces__msg__TeachbotPistolState *
teachbot_interfaces__msg__TeachbotPistolState__create();

/// Destroy msg/TeachbotPistolState message.
/**
 * It calls
 * teachbot_interfaces__msg__TeachbotPistolState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_teachbot_interfaces
void
teachbot_interfaces__msg__TeachbotPistolState__destroy(teachbot_interfaces__msg__TeachbotPistolState * msg);

/// Check for msg/TeachbotPistolState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_teachbot_interfaces
bool
teachbot_interfaces__msg__TeachbotPistolState__are_equal(const teachbot_interfaces__msg__TeachbotPistolState * lhs, const teachbot_interfaces__msg__TeachbotPistolState * rhs);

/// Copy a msg/TeachbotPistolState message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_teachbot_interfaces
bool
teachbot_interfaces__msg__TeachbotPistolState__copy(
  const teachbot_interfaces__msg__TeachbotPistolState * input,
  teachbot_interfaces__msg__TeachbotPistolState * output);

/// Initialize array of msg/TeachbotPistolState messages.
/**
 * It allocates the memory for the number of elements and calls
 * teachbot_interfaces__msg__TeachbotPistolState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_teachbot_interfaces
bool
teachbot_interfaces__msg__TeachbotPistolState__Sequence__init(teachbot_interfaces__msg__TeachbotPistolState__Sequence * array, size_t size);

/// Finalize array of msg/TeachbotPistolState messages.
/**
 * It calls
 * teachbot_interfaces__msg__TeachbotPistolState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_teachbot_interfaces
void
teachbot_interfaces__msg__TeachbotPistolState__Sequence__fini(teachbot_interfaces__msg__TeachbotPistolState__Sequence * array);

/// Create array of msg/TeachbotPistolState messages.
/**
 * It allocates the memory for the array and calls
 * teachbot_interfaces__msg__TeachbotPistolState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_teachbot_interfaces
teachbot_interfaces__msg__TeachbotPistolState__Sequence *
teachbot_interfaces__msg__TeachbotPistolState__Sequence__create(size_t size);

/// Destroy array of msg/TeachbotPistolState messages.
/**
 * It calls
 * teachbot_interfaces__msg__TeachbotPistolState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_teachbot_interfaces
void
teachbot_interfaces__msg__TeachbotPistolState__Sequence__destroy(teachbot_interfaces__msg__TeachbotPistolState__Sequence * array);

/// Check for msg/TeachbotPistolState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_teachbot_interfaces
bool
teachbot_interfaces__msg__TeachbotPistolState__Sequence__are_equal(const teachbot_interfaces__msg__TeachbotPistolState__Sequence * lhs, const teachbot_interfaces__msg__TeachbotPistolState__Sequence * rhs);

/// Copy an array of msg/TeachbotPistolState messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_teachbot_interfaces
bool
teachbot_interfaces__msg__TeachbotPistolState__Sequence__copy(
  const teachbot_interfaces__msg__TeachbotPistolState__Sequence * input,
  teachbot_interfaces__msg__TeachbotPistolState__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TEACHBOT_INTERFACES__MSG__DETAIL__TEACHBOT_PISTOL_STATE__FUNCTIONS_H_
