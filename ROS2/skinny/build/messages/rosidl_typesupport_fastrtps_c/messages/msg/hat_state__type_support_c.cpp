// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from messages:msg/HatState.idl
// generated code does not contain a copyright notice
#include "messages/msg/hat_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "messages/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "messages/msg/hat_state__struct.h"
#include "messages/msg/hat_state__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _HatState__ros_msg_type = messages__msg__HatState;

static bool _HatState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _HatState__ros_msg_type * ros_message = static_cast<const _HatState__ros_msg_type *>(untyped_ros_message);
  // Field name: joystick
  {
    cdr << ros_message->joystick;
  }

  // Field name: hat
  {
    cdr << ros_message->hat;
  }

  // Field name: state
  {
    cdr << ros_message->state;
  }

  return true;
}

static bool _HatState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _HatState__ros_msg_type * ros_message = static_cast<_HatState__ros_msg_type *>(untyped_ros_message);
  // Field name: joystick
  {
    cdr >> ros_message->joystick;
  }

  // Field name: hat
  {
    cdr >> ros_message->hat;
  }

  // Field name: state
  {
    cdr >> ros_message->state;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_messages
size_t get_serialized_size_messages__msg__HatState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _HatState__ros_msg_type * ros_message = static_cast<const _HatState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name joystick
  {
    size_t item_size = sizeof(ros_message->joystick);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name hat
  {
    size_t item_size = sizeof(ros_message->hat);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name state
  {
    size_t item_size = sizeof(ros_message->state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _HatState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_messages__msg__HatState(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_messages
size_t max_serialized_size_messages__msg__HatState(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: joystick
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: hat
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: state
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }

  return current_alignment - initial_alignment;
}

static size_t _HatState__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_messages__msg__HatState(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_HatState = {
  "messages::msg",
  "HatState",
  _HatState__cdr_serialize,
  _HatState__cdr_deserialize,
  _HatState__get_serialized_size,
  _HatState__max_serialized_size
};

static rosidl_message_type_support_t _HatState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_HatState,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, messages, msg, HatState)() {
  return &_HatState__type_support;
}

#if defined(__cplusplus)
}
#endif
