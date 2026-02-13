// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from teachbot_interfaces:msg/TeachbotPistolState.idl
// generated code does not contain a copyright notice
#include "teachbot_interfaces/msg/detail/teachbot_pistol_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
teachbot_interfaces__msg__TeachbotPistolState__init(teachbot_interfaces__msg__TeachbotPistolState * msg)
{
  if (!msg) {
    return false;
  }
  // pot_raw
  // pot_percent
  // btn1
  // btn2
  return true;
}

void
teachbot_interfaces__msg__TeachbotPistolState__fini(teachbot_interfaces__msg__TeachbotPistolState * msg)
{
  if (!msg) {
    return;
  }
  // pot_raw
  // pot_percent
  // btn1
  // btn2
}

bool
teachbot_interfaces__msg__TeachbotPistolState__are_equal(const teachbot_interfaces__msg__TeachbotPistolState * lhs, const teachbot_interfaces__msg__TeachbotPistolState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pot_raw
  if (lhs->pot_raw != rhs->pot_raw) {
    return false;
  }
  // pot_percent
  if (lhs->pot_percent != rhs->pot_percent) {
    return false;
  }
  // btn1
  if (lhs->btn1 != rhs->btn1) {
    return false;
  }
  // btn2
  if (lhs->btn2 != rhs->btn2) {
    return false;
  }
  return true;
}

bool
teachbot_interfaces__msg__TeachbotPistolState__copy(
  const teachbot_interfaces__msg__TeachbotPistolState * input,
  teachbot_interfaces__msg__TeachbotPistolState * output)
{
  if (!input || !output) {
    return false;
  }
  // pot_raw
  output->pot_raw = input->pot_raw;
  // pot_percent
  output->pot_percent = input->pot_percent;
  // btn1
  output->btn1 = input->btn1;
  // btn2
  output->btn2 = input->btn2;
  return true;
}

teachbot_interfaces__msg__TeachbotPistolState *
teachbot_interfaces__msg__TeachbotPistolState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teachbot_interfaces__msg__TeachbotPistolState * msg = (teachbot_interfaces__msg__TeachbotPistolState *)allocator.allocate(sizeof(teachbot_interfaces__msg__TeachbotPistolState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(teachbot_interfaces__msg__TeachbotPistolState));
  bool success = teachbot_interfaces__msg__TeachbotPistolState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
teachbot_interfaces__msg__TeachbotPistolState__destroy(teachbot_interfaces__msg__TeachbotPistolState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    teachbot_interfaces__msg__TeachbotPistolState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
teachbot_interfaces__msg__TeachbotPistolState__Sequence__init(teachbot_interfaces__msg__TeachbotPistolState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teachbot_interfaces__msg__TeachbotPistolState * data = NULL;

  if (size) {
    data = (teachbot_interfaces__msg__TeachbotPistolState *)allocator.zero_allocate(size, sizeof(teachbot_interfaces__msg__TeachbotPistolState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = teachbot_interfaces__msg__TeachbotPistolState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        teachbot_interfaces__msg__TeachbotPistolState__fini(&data[i - 1]);
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
teachbot_interfaces__msg__TeachbotPistolState__Sequence__fini(teachbot_interfaces__msg__TeachbotPistolState__Sequence * array)
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
      teachbot_interfaces__msg__TeachbotPistolState__fini(&array->data[i]);
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

teachbot_interfaces__msg__TeachbotPistolState__Sequence *
teachbot_interfaces__msg__TeachbotPistolState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  teachbot_interfaces__msg__TeachbotPistolState__Sequence * array = (teachbot_interfaces__msg__TeachbotPistolState__Sequence *)allocator.allocate(sizeof(teachbot_interfaces__msg__TeachbotPistolState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = teachbot_interfaces__msg__TeachbotPistolState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
teachbot_interfaces__msg__TeachbotPistolState__Sequence__destroy(teachbot_interfaces__msg__TeachbotPistolState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    teachbot_interfaces__msg__TeachbotPistolState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
teachbot_interfaces__msg__TeachbotPistolState__Sequence__are_equal(const teachbot_interfaces__msg__TeachbotPistolState__Sequence * lhs, const teachbot_interfaces__msg__TeachbotPistolState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!teachbot_interfaces__msg__TeachbotPistolState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
teachbot_interfaces__msg__TeachbotPistolState__Sequence__copy(
  const teachbot_interfaces__msg__TeachbotPistolState__Sequence * input,
  teachbot_interfaces__msg__TeachbotPistolState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(teachbot_interfaces__msg__TeachbotPistolState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    teachbot_interfaces__msg__TeachbotPistolState * data =
      (teachbot_interfaces__msg__TeachbotPistolState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!teachbot_interfaces__msg__TeachbotPistolState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          teachbot_interfaces__msg__TeachbotPistolState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!teachbot_interfaces__msg__TeachbotPistolState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
