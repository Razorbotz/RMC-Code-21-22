// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from messages:msg/TalonOut.idl
// generated code does not contain a copyright notice
#include "messages/msg/talon_out__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "messages/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "messages/msg/talon_out__struct.h"
#include "messages/msg/talon_out__functions.h"
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


using _TalonOut__ros_msg_type = messages__msg__TalonOut;

static bool _TalonOut__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _TalonOut__ros_msg_type * ros_message = static_cast<const _TalonOut__ros_msg_type *>(untyped_ros_message);
  // Field name: device_id
  {
    cdr << ros_message->device_id;
  }

  // Field name: bus_voltage
  {
    cdr << ros_message->bus_voltage;
  }

  // Field name: output_current
  {
    cdr << ros_message->output_current;
  }

  // Field name: output_voltage
  {
    cdr << ros_message->output_voltage;
  }

  // Field name: output_percent
  {
    cdr << ros_message->output_percent;
  }

  // Field name: temperature
  {
    cdr << ros_message->temperature;
  }

  // Field name: sensor_position
  {
    cdr << ros_message->sensor_position;
  }

  // Field name: sensor_velocity
  {
    cdr << ros_message->sensor_velocity;
  }

  // Field name: closed_loop_error
  {
    cdr << ros_message->closed_loop_error;
  }

  // Field name: integral_accumulator
  {
    cdr << ros_message->integral_accumulator;
  }

  // Field name: error_derivative
  {
    cdr << ros_message->error_derivative;
  }

  return true;
}

static bool _TalonOut__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _TalonOut__ros_msg_type * ros_message = static_cast<_TalonOut__ros_msg_type *>(untyped_ros_message);
  // Field name: device_id
  {
    cdr >> ros_message->device_id;
  }

  // Field name: bus_voltage
  {
    cdr >> ros_message->bus_voltage;
  }

  // Field name: output_current
  {
    cdr >> ros_message->output_current;
  }

  // Field name: output_voltage
  {
    cdr >> ros_message->output_voltage;
  }

  // Field name: output_percent
  {
    cdr >> ros_message->output_percent;
  }

  // Field name: temperature
  {
    cdr >> ros_message->temperature;
  }

  // Field name: sensor_position
  {
    cdr >> ros_message->sensor_position;
  }

  // Field name: sensor_velocity
  {
    cdr >> ros_message->sensor_velocity;
  }

  // Field name: closed_loop_error
  {
    cdr >> ros_message->closed_loop_error;
  }

  // Field name: integral_accumulator
  {
    cdr >> ros_message->integral_accumulator;
  }

  // Field name: error_derivative
  {
    cdr >> ros_message->error_derivative;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_messages
size_t get_serialized_size_messages__msg__TalonOut(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _TalonOut__ros_msg_type * ros_message = static_cast<const _TalonOut__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name device_id
  {
    size_t item_size = sizeof(ros_message->device_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name bus_voltage
  {
    size_t item_size = sizeof(ros_message->bus_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name output_current
  {
    size_t item_size = sizeof(ros_message->output_current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name output_voltage
  {
    size_t item_size = sizeof(ros_message->output_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name output_percent
  {
    size_t item_size = sizeof(ros_message->output_percent);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name temperature
  {
    size_t item_size = sizeof(ros_message->temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name sensor_position
  {
    size_t item_size = sizeof(ros_message->sensor_position);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name sensor_velocity
  {
    size_t item_size = sizeof(ros_message->sensor_velocity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name closed_loop_error
  {
    size_t item_size = sizeof(ros_message->closed_loop_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name integral_accumulator
  {
    size_t item_size = sizeof(ros_message->integral_accumulator);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name error_derivative
  {
    size_t item_size = sizeof(ros_message->error_derivative);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _TalonOut__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_messages__msg__TalonOut(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_messages
size_t max_serialized_size_messages__msg__TalonOut(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: device_id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: bus_voltage
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: output_current
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: output_voltage
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: output_percent
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: temperature
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: sensor_position
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: sensor_velocity
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: closed_loop_error
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: integral_accumulator
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: error_derivative
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _TalonOut__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_messages__msg__TalonOut(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_TalonOut = {
  "messages::msg",
  "TalonOut",
  _TalonOut__cdr_serialize,
  _TalonOut__cdr_deserialize,
  _TalonOut__get_serialized_size,
  _TalonOut__max_serialized_size
};

static rosidl_message_type_support_t _TalonOut__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_TalonOut,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, messages, msg, TalonOut)() {
  return &_TalonOut__type_support;
}

#if defined(__cplusplus)
}
#endif
