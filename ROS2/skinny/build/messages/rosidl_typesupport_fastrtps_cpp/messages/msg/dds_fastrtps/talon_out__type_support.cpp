// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from messages:msg/TalonOut.idl
// generated code does not contain a copyright notice
#include "messages/msg/talon_out__rosidl_typesupport_fastrtps_cpp.hpp"
#include "messages/msg/talon_out__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace messages
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_messages
cdr_serialize(
  const messages::msg::TalonOut & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: device_id
  cdr << ros_message.device_id;
  // Member: bus_voltage
  cdr << ros_message.bus_voltage;
  // Member: output_current
  cdr << ros_message.output_current;
  // Member: output_voltage
  cdr << ros_message.output_voltage;
  // Member: output_percent
  cdr << ros_message.output_percent;
  // Member: temperature
  cdr << ros_message.temperature;
  // Member: sensor_position
  cdr << ros_message.sensor_position;
  // Member: sensor_velocity
  cdr << ros_message.sensor_velocity;
  // Member: closed_loop_error
  cdr << ros_message.closed_loop_error;
  // Member: integral_accumulator
  cdr << ros_message.integral_accumulator;
  // Member: error_derivative
  cdr << ros_message.error_derivative;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_messages
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  messages::msg::TalonOut & ros_message)
{
  // Member: device_id
  cdr >> ros_message.device_id;

  // Member: bus_voltage
  cdr >> ros_message.bus_voltage;

  // Member: output_current
  cdr >> ros_message.output_current;

  // Member: output_voltage
  cdr >> ros_message.output_voltage;

  // Member: output_percent
  cdr >> ros_message.output_percent;

  // Member: temperature
  cdr >> ros_message.temperature;

  // Member: sensor_position
  cdr >> ros_message.sensor_position;

  // Member: sensor_velocity
  cdr >> ros_message.sensor_velocity;

  // Member: closed_loop_error
  cdr >> ros_message.closed_loop_error;

  // Member: integral_accumulator
  cdr >> ros_message.integral_accumulator;

  // Member: error_derivative
  cdr >> ros_message.error_derivative;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_messages
get_serialized_size(
  const messages::msg::TalonOut & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: device_id
  {
    size_t item_size = sizeof(ros_message.device_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: bus_voltage
  {
    size_t item_size = sizeof(ros_message.bus_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: output_current
  {
    size_t item_size = sizeof(ros_message.output_current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: output_voltage
  {
    size_t item_size = sizeof(ros_message.output_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: output_percent
  {
    size_t item_size = sizeof(ros_message.output_percent);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: temperature
  {
    size_t item_size = sizeof(ros_message.temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: sensor_position
  {
    size_t item_size = sizeof(ros_message.sensor_position);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: sensor_velocity
  {
    size_t item_size = sizeof(ros_message.sensor_velocity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: closed_loop_error
  {
    size_t item_size = sizeof(ros_message.closed_loop_error);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: integral_accumulator
  {
    size_t item_size = sizeof(ros_message.integral_accumulator);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: error_derivative
  {
    size_t item_size = sizeof(ros_message.error_derivative);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_messages
max_serialized_size_TalonOut(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: device_id
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: bus_voltage
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: output_current
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: output_voltage
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: output_percent
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: temperature
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: sensor_position
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: sensor_velocity
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: closed_loop_error
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: integral_accumulator
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: error_derivative
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  return current_alignment - initial_alignment;
}

static bool _TalonOut__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const messages::msg::TalonOut *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _TalonOut__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<messages::msg::TalonOut *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _TalonOut__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const messages::msg::TalonOut *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _TalonOut__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_TalonOut(full_bounded, 0);
}

static message_type_support_callbacks_t _TalonOut__callbacks = {
  "messages::msg",
  "TalonOut",
  _TalonOut__cdr_serialize,
  _TalonOut__cdr_deserialize,
  _TalonOut__get_serialized_size,
  _TalonOut__max_serialized_size
};

static rosidl_message_type_support_t _TalonOut__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_TalonOut__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace messages

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_messages
const rosidl_message_type_support_t *
get_message_type_support_handle<messages::msg::TalonOut>()
{
  return &messages::msg::typesupport_fastrtps_cpp::_TalonOut__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, messages, msg, TalonOut)() {
  return &messages::msg::typesupport_fastrtps_cpp::_TalonOut__handle;
}

#ifdef __cplusplus
}
#endif
