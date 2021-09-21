// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from messages:msg/Power.idl
// generated code does not contain a copyright notice
#include "messages/msg/power__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "messages/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "messages/msg/power__struct.h"
#include "messages/msg/power__functions.h"
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


using _Power__ros_msg_type = messages__msg__Power;

static bool _Power__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _Power__ros_msg_type * ros_message = static_cast<const _Power__ros_msg_type *>(untyped_ros_message);
  // Field name: voltage
  {
    cdr << ros_message->voltage;
  }

  // Field name: current0
  {
    cdr << ros_message->current0;
  }

  // Field name: current1
  {
    cdr << ros_message->current1;
  }

  // Field name: current2
  {
    cdr << ros_message->current2;
  }

  // Field name: current3
  {
    cdr << ros_message->current3;
  }

  // Field name: current4
  {
    cdr << ros_message->current4;
  }

  // Field name: current5
  {
    cdr << ros_message->current5;
  }

  // Field name: current6
  {
    cdr << ros_message->current6;
  }

  // Field name: current7
  {
    cdr << ros_message->current7;
  }

  // Field name: current8
  {
    cdr << ros_message->current8;
  }

  // Field name: current9
  {
    cdr << ros_message->current9;
  }

  // Field name: current10
  {
    cdr << ros_message->current10;
  }

  // Field name: current11
  {
    cdr << ros_message->current11;
  }

  // Field name: current12
  {
    cdr << ros_message->current12;
  }

  // Field name: current13
  {
    cdr << ros_message->current13;
  }

  // Field name: current14
  {
    cdr << ros_message->current14;
  }

  // Field name: current15
  {
    cdr << ros_message->current15;
  }

  return true;
}

static bool _Power__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _Power__ros_msg_type * ros_message = static_cast<_Power__ros_msg_type *>(untyped_ros_message);
  // Field name: voltage
  {
    cdr >> ros_message->voltage;
  }

  // Field name: current0
  {
    cdr >> ros_message->current0;
  }

  // Field name: current1
  {
    cdr >> ros_message->current1;
  }

  // Field name: current2
  {
    cdr >> ros_message->current2;
  }

  // Field name: current3
  {
    cdr >> ros_message->current3;
  }

  // Field name: current4
  {
    cdr >> ros_message->current4;
  }

  // Field name: current5
  {
    cdr >> ros_message->current5;
  }

  // Field name: current6
  {
    cdr >> ros_message->current6;
  }

  // Field name: current7
  {
    cdr >> ros_message->current7;
  }

  // Field name: current8
  {
    cdr >> ros_message->current8;
  }

  // Field name: current9
  {
    cdr >> ros_message->current9;
  }

  // Field name: current10
  {
    cdr >> ros_message->current10;
  }

  // Field name: current11
  {
    cdr >> ros_message->current11;
  }

  // Field name: current12
  {
    cdr >> ros_message->current12;
  }

  // Field name: current13
  {
    cdr >> ros_message->current13;
  }

  // Field name: current14
  {
    cdr >> ros_message->current14;
  }

  // Field name: current15
  {
    cdr >> ros_message->current15;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_messages
size_t get_serialized_size_messages__msg__Power(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _Power__ros_msg_type * ros_message = static_cast<const _Power__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name voltage
  {
    size_t item_size = sizeof(ros_message->voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current0
  {
    size_t item_size = sizeof(ros_message->current0);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current1
  {
    size_t item_size = sizeof(ros_message->current1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current2
  {
    size_t item_size = sizeof(ros_message->current2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current3
  {
    size_t item_size = sizeof(ros_message->current3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current4
  {
    size_t item_size = sizeof(ros_message->current4);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current5
  {
    size_t item_size = sizeof(ros_message->current5);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current6
  {
    size_t item_size = sizeof(ros_message->current6);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current7
  {
    size_t item_size = sizeof(ros_message->current7);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current8
  {
    size_t item_size = sizeof(ros_message->current8);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current9
  {
    size_t item_size = sizeof(ros_message->current9);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current10
  {
    size_t item_size = sizeof(ros_message->current10);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current11
  {
    size_t item_size = sizeof(ros_message->current11);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current12
  {
    size_t item_size = sizeof(ros_message->current12);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current13
  {
    size_t item_size = sizeof(ros_message->current13);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current14
  {
    size_t item_size = sizeof(ros_message->current14);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current15
  {
    size_t item_size = sizeof(ros_message->current15);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _Power__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_messages__msg__Power(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_messages
size_t max_serialized_size_messages__msg__Power(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: voltage
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: current0
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: current1
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: current2
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: current3
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: current4
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: current5
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: current6
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: current7
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: current8
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: current9
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: current10
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: current11
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: current12
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: current13
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: current14
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: current15
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _Power__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_messages__msg__Power(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_Power = {
  "messages::msg",
  "Power",
  _Power__cdr_serialize,
  _Power__cdr_deserialize,
  _Power__get_serialized_size,
  _Power__max_serialized_size
};

static rosidl_message_type_support_t _Power__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_Power,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, messages, msg, Power)() {
  return &_Power__type_support;
}

#if defined(__cplusplus)
}
#endif
