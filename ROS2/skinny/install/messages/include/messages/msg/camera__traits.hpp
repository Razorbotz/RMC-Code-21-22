// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from messages:msg/Camera.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__CAMERA__TRAITS_HPP_
#define MESSAGES__MSG__CAMERA__TRAITS_HPP_

#include "messages/msg/camera__struct.hpp"
#include <rosidl_generator_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<messages::msg::Camera>()
{
  return "messages::msg::Camera";
}

template<>
struct has_fixed_size<messages::msg::Camera>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<messages::msg::Camera>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<messages::msg::Camera>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MESSAGES__MSG__CAMERA__TRAITS_HPP_
