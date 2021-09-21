// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from messages:msg/ZedPosition.idl
// generated code does not contain a copyright notice
#include "messages/msg/zed_position__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
messages__msg__ZedPosition__init(messages__msg__ZedPosition * msg)
{
  if (!msg) {
    return false;
  }
  // x
  // y
  // z
  // ox
  // oy
  // oz
  // ow
  // aruco_visible
  return true;
}

void
messages__msg__ZedPosition__fini(messages__msg__ZedPosition * msg)
{
  if (!msg) {
    return;
  }
  // x
  // y
  // z
  // ox
  // oy
  // oz
  // ow
  // aruco_visible
}

messages__msg__ZedPosition *
messages__msg__ZedPosition__create()
{
  messages__msg__ZedPosition * msg = (messages__msg__ZedPosition *)malloc(sizeof(messages__msg__ZedPosition));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(messages__msg__ZedPosition));
  bool success = messages__msg__ZedPosition__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
messages__msg__ZedPosition__destroy(messages__msg__ZedPosition * msg)
{
  if (msg) {
    messages__msg__ZedPosition__fini(msg);
  }
  free(msg);
}


bool
messages__msg__ZedPosition__Sequence__init(messages__msg__ZedPosition__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  messages__msg__ZedPosition * data = NULL;
  if (size) {
    data = (messages__msg__ZedPosition *)calloc(size, sizeof(messages__msg__ZedPosition));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = messages__msg__ZedPosition__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        messages__msg__ZedPosition__fini(&data[i - 1]);
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
messages__msg__ZedPosition__Sequence__fini(messages__msg__ZedPosition__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      messages__msg__ZedPosition__fini(&array->data[i]);
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

messages__msg__ZedPosition__Sequence *
messages__msg__ZedPosition__Sequence__create(size_t size)
{
  messages__msg__ZedPosition__Sequence * array = (messages__msg__ZedPosition__Sequence *)malloc(sizeof(messages__msg__ZedPosition__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = messages__msg__ZedPosition__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
messages__msg__ZedPosition__Sequence__destroy(messages__msg__ZedPosition__Sequence * array)
{
  if (array) {
    messages__msg__ZedPosition__Sequence__fini(array);
  }
  free(array);
}
