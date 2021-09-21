// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from messages:msg/ZedPosition.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__ZED_POSITION__STRUCT_H_
#define MESSAGES__MSG__ZED_POSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/ZedPosition in the package messages.
typedef struct messages__msg__ZedPosition
{
  float x;
  float y;
  float z;
  float ox;
  float oy;
  float oz;
  float ow;
  bool aruco_visible;
} messages__msg__ZedPosition;

// Struct for a sequence of messages__msg__ZedPosition.
typedef struct messages__msg__ZedPosition__Sequence
{
  messages__msg__ZedPosition * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} messages__msg__ZedPosition__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MESSAGES__MSG__ZED_POSITION__STRUCT_H_
