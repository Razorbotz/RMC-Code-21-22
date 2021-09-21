// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from messages:msg/JoystickCommand.idl
// generated code does not contain a copyright notice
#include "messages/msg/joystick_command__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "messages/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "messages/msg/joystick_command__struct.h"
#include "messages/msg/joystick_command__functions.h"
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


using _JoystickCommand__ros_msg_type = messages__msg__JoystickCommand;

static bool _JoystickCommand__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _JoystickCommand__ros_msg_type * ros_message = static_cast<const _JoystickCommand__ros_msg_type *>(untyped_ros_message);
  // Field name: number
  {
    cdr << ros_message->number;
  }

  // Field name: element
  {
    cdr << ros_message->element;
  }

  // Field name: value
  {
    cdr << ros_message->value;
  }

  return true;
}

static bool _JoystickCommand__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _JoystickCommand__ros_msg_type * ros_message = static_cast<_JoystickCommand__ros_msg_type *>(untyped_ros_message);
  // Field name: number
  {
    cdr >> ros_message->number;
  }

  // Field name: element
  {
    cdr >> ros_message->element;
  }

  // Field name: value
  {
    cdr >> ros_message->value;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_messages
size_t get_serialized_size_messages__msg__JoystickCommand(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _JoystickCommand__ros_msg_type * ros_message = static_cast<const _JoystickCommand__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name number
  {
    size_t item_size = sizeof(ros_message->number);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name element
  {
    size_t item_size = sizeof(ros_message->element);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name value
  {
    size_t item_size = sizeof(ros_message->value);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _JoystickCommand__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_messages__msg__JoystickCommand(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_messages
size_t max_serialized_size_messages__msg__JoystickCommand(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: number
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: element
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: value
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _JoystickCommand__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_messages__msg__JoystickCommand(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_JoystickCommand = {
  "messages::msg",
  "JoystickCommand",
  _JoystickCommand__cdr_serialize,
  _JoystickCommand__cdr_deserialize,
  _JoystickCommand__get_serialized_size,
  _JoystickCommand__max_serialized_size
};

static rosidl_message_type_support_t _JoystickCommand__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_JoystickCommand,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, messages, msg, JoystickCommand)() {
  return &_JoystickCommand__type_support;
}

#if defined(__cplusplus)
}
#endif
