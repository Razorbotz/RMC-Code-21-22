// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from messages:msg/ButtonState.idl
// generated code does not contain a copyright notice
#include "messages/msg/button_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
messages__msg__ButtonState__init(messages__msg__ButtonState * msg)
{
  if (!msg) {
    return false;
  }
  // joystick
  // button
  // state
  return true;
}

void
messages__msg__ButtonState__fini(messages__msg__ButtonState * msg)
{
  if (!msg) {
    return;
  }
  // joystick
  // button
  // state
}

messages__msg__ButtonState *
messages__msg__ButtonState__create()
{
  messages__msg__ButtonState * msg = (messages__msg__ButtonState *)malloc(sizeof(messages__msg__ButtonState));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(messages__msg__ButtonState));
  bool success = messages__msg__ButtonState__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
messages__msg__ButtonState__destroy(messages__msg__ButtonState * msg)
{
  if (msg) {
    messages__msg__ButtonState__fini(msg);
  }
  free(msg);
}


bool
messages__msg__ButtonState__Sequence__init(messages__msg__ButtonState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  messages__msg__ButtonState * data = NULL;
  if (size) {
    data = (messages__msg__ButtonState *)calloc(size, sizeof(messages__msg__ButtonState));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = messages__msg__ButtonState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        messages__msg__ButtonState__fini(&data[i - 1]);
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
messages__msg__ButtonState__Sequence__fini(messages__msg__ButtonState__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      messages__msg__ButtonState__fini(&array->data[i]);
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

messages__msg__ButtonState__Sequence *
messages__msg__ButtonState__Sequence__create(size_t size)
{
  messages__msg__ButtonState__Sequence * array = (messages__msg__ButtonState__Sequence *)malloc(sizeof(messages__msg__ButtonState__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = messages__msg__ButtonState__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
messages__msg__ButtonState__Sequence__destroy(messages__msg__ButtonState__Sequence * array)
{
  if (array) {
    messages__msg__ButtonState__Sequence__fini(array);
  }
  free(array);
}
