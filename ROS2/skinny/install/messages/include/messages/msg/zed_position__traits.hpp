// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from messages:msg/ZedPosition.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__ZED_POSITION__TRAITS_HPP_
#define MESSAGES__MSG__ZED_POSITION__TRAITS_HPP_

#include "messages/msg/zed_position__struct.hpp"
#include <rosidl_generator_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<messages::msg::ZedPosition>()
{
  return "messages::msg::ZedPosition";
}

template<>
struct has_fixed_size<messages::msg::ZedPosition>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<messages::msg::ZedPosition>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<messages::msg::ZedPosition>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESSAGES__MSG__ZED_POSITION__TRAITS_HPP_
