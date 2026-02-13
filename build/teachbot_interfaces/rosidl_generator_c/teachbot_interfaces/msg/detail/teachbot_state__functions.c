// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from teachbot_interfaces:msg/TeachbotState.idl
// generated code does not contain a copyright notice
#include "teachbot_interfaces/msg/detail/teachbot_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `pistol`
#include "teachbot_interfaces/msg/detail/teachbot_pistol_state__functions.h"
// Member `robot_model`
#include "rosidl_runtime_c/string_functions.h"

bool
teachbot_interfaces__msg__TeachbotState__init(teachbot_interfaces__msg__TeachbotState * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    teachbot_interfaces__msg__TeachbotState__fini(msg);
    return false;
  }
  // joint_angles_deg
  // tcp_x
  // tcp_y
  // tcp_z
  // tcp_rx
  // tcp_ry
  // tcp_rz
  // pistol
  if (!teachbot_interfaces__msg__TeachbotPistolState__init(&msg->pistol)) {
    teachbot_interfaces__msg__TeachbotState__fini(msg);
    return false;
  }
  // encoder_errors
  // encoder_warnings
  // encoder_frequencies
  // robot_model
  if (!rosidl_runtime_c__String__init(&msg->robot_model)) {
    teachbot_interfaces__msg__TeachbotState__fini(msg);
    return false;
  }
  return true;
}

void
teachbot_interfaces__msg__TeachbotState__fini(teachbot_interfaces__msg__TeachbotState * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // joint_angles_deg
  // tcp_x
  // tcp_y
  // tcp_z
  // tcp_rx
  // tcp_ry
  // tcp_rz
  // pistol
  teachbot_interfaces__msg__TeachbotPistolState__fini(&msg->pistol);
  // encoder_errors
  // encoder_warnings
  // encoder_frequencies
  // robot_model
  rosidl_runtime_c__String__fini(&msg->robot_model);
}

bool
teachbot_interfaces__msg__TeachbotState__are_equal(const teachbot_interfaces__msg__TeachbotState * lhs, const teachbot_interfaces__msg__TeachbotState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // joint_angles_deg
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->joint_angles_deg[i] != rhs->joint_angles_deg[i]) {
      return false;
    }
  }
  // tcp_x
  if (lhs->tcp_x != rhs->tcp_x) {
    return false;
  }
  // tcp_y
  if (lhs->tcp_y != rhs->tcp_y) {
    return false;
  }
  // tcp_z
  if (lhs->tcp_z != rhs->tcp_z) {
    return false;
  }
  // tcp_rx
  if (lhs->tcp_rx != rhs->tcp_rx) {
    return false;
  }
  // tcp_ry
  if (lhs->tcp_ry != rhs->tcp_ry) {
    return false;
  }
  // tcp_rz
  if (lhs->tcp_rz != rhs->tcp_rz) {
    return false;
  }
  // pistol
  if (!teachbot_interfaces__msg__TeachbotPistolState__are_equal(
      &(lhs->pistol), &(rhs->pistol)))
  {
    return false;
  }
  // encoder_errors
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->encoder_errors[i] != rhs->encoder_errors[i]) {
      return false;
    }
  }
  // encoder_warnings
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->encoder_warnings[i] != rhs->encoder_warnings[i]) {
      return false;
    }
  }
  // encoder_frequencies
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->encoder_frequencies[i] != rhs->encoder_frequencies[i]) {
      return false;
    }
  }
  // robot_model
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->robot_model), &(rhs->robot_model)))
  {
    return false;
  }
  return true;
}

bool
teachbot_interfaces__msg__TeachbotState__copy(
  const teachbot_interfaces__msg__TeachbotState * input,
  teachbot_interfaces__msg__TeachbotState * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // joint_angles_deg
  for (size_t i = 0; i < 6; ++i) {
    output->joint_angles_deg[i] = input->joint_angles_deg[i];
  }
  // tcp_x
  output->tcp_x = input->tcp_x;
  // tcp_y
  output->tcp_y = input->tcp_y;
  // tcp_z
  output->tcp_z = input->tcp_z;
  // tcp_rx
  output->tcp_rx = input->tcp_rx;
  // tcp_ry
  output->tcp_ry = input->tcp_ry;
  // tcp_rz
  output->tcp_rz = input->tcp_rz;
  // pistol
  if (!teachbot_interfaces__msg__TeachbotPistolState__copy(
      &(input->pistol), &(output->pistol)))
  {
    return false;
  }
  // encoder_errors
  for (size_t i = 0; i < 6; ++i) {
    output->encoder_errors[i] = input->encoder_errors[i];
  }
  // encoder_warnings
  for (size_t i = 0; i < 6; ++i) {
    output->encoder_warnings[i] = input->encoder_warnings[i];
  }
  // encoder_frequencies
  for (size_t i = 0; i < 6; ++i) {
    output->encoder_frequencies[i] = input->encoder_frequencies[i];
  }
  // robot_model
  if (!rosidl_runtime_c__String__copy(
      &(input->robot_model), &(output->robot_model)))
  {
    return false;
  }
  return true;
}

teachbot_interfaces__msg__TeachbotState *
teachbot_interfaces__msg__TeachbotState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teachbot_interfaces__msg__TeachbotState * msg = (teachbot_interfaces__msg__TeachbotState *)allocator.allocate(sizeof(teachbot_interfaces__msg__TeachbotState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(teachbot_interfaces__msg__TeachbotState));
  bool success = teachbot_interfaces__msg__TeachbotState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
teachbot_interfaces__msg__TeachbotState__destroy(teachbot_interfaces__msg__TeachbotState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    teachbot_interfaces__msg__TeachbotState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
teachbot_interfaces__msg__TeachbotState__Sequence__init(teachbot_interfaces__msg__TeachbotState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teachbot_interfaces__msg__TeachbotState * data = NULL;

  if (size) {
    data = (teachbot_interfaces__msg__TeachbotState *)allocator.zero_allocate(size, sizeof(teachbot_interfaces__msg__TeachbotState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = teachbot_interfaces__msg__TeachbotState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        teachbot_interfaces__msg__TeachbotState__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
teachbot_interfaces__msg__TeachbotState__Sequence__fini(teachbot_interfaces__msg__TeachbotState__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      teachbot_interfaces__msg__TeachbotState__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

teachbot_interfaces__msg__TeachbotState__Sequence *
teachbot_interfaces__msg__TeachbotState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teachbot_interfaces__msg__TeachbotState__Sequence * array = (teachbot_interfaces__msg__TeachbotState__Sequence *)allocator.allocate(sizeof(teachbot_interfaces__msg__TeachbotState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = teachbot_interfaces__msg__TeachbotState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
teachbot_interfaces__msg__TeachbotState__Sequence__destroy(teachbot_interfaces__msg__TeachbotState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    teachbot_interfaces__msg__TeachbotState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
teachbot_interfaces__msg__TeachbotState__Sequence__are_equal(const teachbot_interfaces__msg__TeachbotState__Sequence * lhs, const teachbot_interfaces__msg__TeachbotState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!teachbot_interfaces__msg__TeachbotState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
teachbot_interfaces__msg__TeachbotState__Sequence__copy(
  const teachbot_interfaces__msg__TeachbotState__Sequence * input,
  teachbot_interfaces__msg__TeachbotState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(teachbot_interfaces__msg__TeachbotState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    teachbot_interfaces__msg__TeachbotState * data =
      (teachbot_interfaces__msg__TeachbotState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!teachbot_interfaces__msg__TeachbotState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          teachbot_interfaces__msg__TeachbotState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!teachbot_interfaces__msg__TeachbotState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
