// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from messages:msg/Power.idl
// generated code does not contain a copyright notice
#include "messages/msg/power__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
messages__msg__Power__init(messages__msg__Power * msg)
{
  if (!msg) {
    return false;
  }
  // voltage
  // current0
  // current1
  // current2
  // current3
  // current4
  // current5
  // current6
  // current7
  // current8
  // current9
  // current10
  // current11
  // current12
  // current13
  // current14
  // current15
  return true;
}

void
messages__msg__Power__fini(messages__msg__Power * msg)
{
  if (!msg) {
    return;
  }
  // voltage
  // current0
  // current1
  // current2
  // current3
  // current4
  // current5
  // current6
  // current7
  // current8
  // current9
  // current10
  // current11
  // current12
  // current13
  // current14
  // current15
}

messages__msg__Power *
messages__msg__Power__create()
{
  messages__msg__Power * msg = (messages__msg__Power *)malloc(sizeof(messages__msg__Power));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(messages__msg__Power));
  bool success = messages__msg__Power__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
messages__msg__Power__destroy(messages__msg__Power * msg)
{
  if (msg) {
    messages__msg__Power__fini(msg);
  }
  free(msg);
}


bool
messages__msg__Power__Sequence__init(messages__msg__Power__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  messages__msg__Power * data = NULL;
  if (size) {
    data = (messages__msg__Power *)calloc(size, sizeof(messages__msg__Power));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = messages__msg__Power__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        messages__msg__Power__fini(&data[i - 1]);
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
messages__msg__Power__Sequence__fini(messages__msg__Power__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      messages__msg__Power__fini(&array->data[i]);
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

messages__msg__Power__Sequence *
messages__msg__Power__Sequence__create(size_t size)
{
  messages__msg__Power__Sequence * array = (messages__msg__Power__Sequence *)malloc(sizeof(messages__msg__Power__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = messages__msg__Power__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
messages__msg__Power__Sequence__destroy(messages__msg__Power__Sequence * array)
{
  if (array) {
    messages__msg__Power__Sequence__fini(array);
  }
  free(array);
}
