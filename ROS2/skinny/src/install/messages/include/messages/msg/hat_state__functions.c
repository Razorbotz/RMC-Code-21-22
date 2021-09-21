// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from messages:msg/HatState.idl
// generated code does not contain a copyright notice
#include "messages/msg/hat_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
messages__msg__HatState__init(messages__msg__HatState * msg)
{
  if (!msg) {
    return false;
  }
  // joystick
  // hat
  // state
  return true;
}

void
messages__msg__HatState__fini(messages__msg__HatState * msg)
{
  if (!msg) {
    return;
  }
  // joystick
  // hat
  // state
}

messages__msg__HatState *
messages__msg__HatState__create()
{
  messages__msg__HatState * msg = (messages__msg__HatState *)malloc(sizeof(messages__msg__HatState));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(messages__msg__HatState));
  bool success = messages__msg__HatState__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
messages__msg__HatState__destroy(messages__msg__HatState * msg)
{
  if (msg) {
    messages__msg__HatState__fini(msg);
  }
  free(msg);
}


bool
messages__msg__HatState__Sequence__init(messages__msg__HatState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  messages__msg__HatState * data = NULL;
  if (size) {
    data = (messages__msg__HatState *)calloc(size, sizeof(messages__msg__HatState));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = messages__msg__HatState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        messages__msg__HatState__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
messages__msg__HatState__Sequence__fini(messages__msg__HatState__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      messages__msg__HatState__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

messages__msg__HatState__Sequence *
messages__msg__HatState__Sequence__create(size_t size)
{
  messages__msg__HatState__Sequence * array = (messages__msg__HatState__Sequence *)malloc(sizeof(messages__msg__HatState__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = messages__msg__HatState__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
messages__msg__HatState__Sequence__destroy(messages__msg__HatState__Sequence * array)
{
  if (array) {
    messages__msg__HatState__Sequence__fini(array);
  }
  free(array);
}
