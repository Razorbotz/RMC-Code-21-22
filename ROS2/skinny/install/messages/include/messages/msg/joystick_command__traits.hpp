// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from messages:msg/JoystickCommand.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__JOYSTICK_COMMAND__TRAITS_HPP_
#define MESSAGES__MSG__JOYSTICK_COMMAND__TRAITS_HPP_

#include "messages/msg/joystick_command__struct.hpp"
#include <rosidl_generator_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<messages::msg::JoystickCommand>()
{
  return "messages::msg::JoystickCommand";
}

template<>
struct has_fixed_size<messages::msg::JoystickCommand>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<messages::msg::JoystickCommand>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<messages::msg::JoystickCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESSAGES__MSG__JOYSTICK_COMMAND__TRAITS_HPP_
