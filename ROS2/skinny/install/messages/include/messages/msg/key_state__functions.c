// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from messages:msg/KeyState.idl
// generated code does not contain a copyright notice
#include "messages/msg/key_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
messages__msg__KeyState__init(messages__msg__KeyState * msg)
{
  if (!msg) {
    return false;
  }
  // key
  // state
  return true;
}

void
messages__msg__KeyState__fini(messages__msg__KeyState * msg)
{
  if (!msg) {
    return;
  }
  // key
  // state
}

messages__msg__KeyState *
messages__msg__KeyState__create()
{
  messages__msg__KeyState * msg = (messages__msg__KeyState *)malloc(sizeof(messages__msg__KeyState));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(messages__msg__KeyState));
  bool success = messages__msg__KeyState__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
messages__msg__KeyState__destroy(messages__msg__KeyState * msg)
{
  if (msg) {
    messages__msg__KeyState__fini(msg);
  }
  free(msg);
}


bool
messages__msg__KeyState__Sequence__init(messages__msg__KeyState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  messages__msg__KeyState * data = NULL;
  if (size) {
    data = (messages__msg__KeyState *)calloc(size, sizeof(messages__msg__KeyState));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = messages__msg__KeyState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        messages__msg__KeyState__fini(&data[i - 1]);
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
messages__msg__KeyState__Sequence__fini(messages__msg__KeyState__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      messages__msg__KeyState__fini(&array->data[i]);
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

messages__msg__KeyState__Sequence *
messages__msg__KeyState__Sequence__create(size_t size)
{
  messages__msg__KeyState__Sequence * array = (messages__msg__KeyState__Sequence *)malloc(sizeof(messages__msg__KeyState__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = messages__msg__KeyState__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
messages__msg__KeyState__Sequence__destroy(messages__msg__KeyState__Sequence * array)
{
  if (array) {
    messages__msg__KeyState__Sequence__fini(array);
  }
  free(array);
}
