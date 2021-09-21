// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from messages:msg/ZedPosition.idl
// generated code does not contain a copyright notice

#ifndef MESSAGES__MSG__ZED_POSITION__STRUCT_HPP_
#define MESSAGES__MSG__ZED_POSITION__STRUCT_HPP_

#include <rosidl_generator_cpp/bounded_vector.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__messages__msg__ZedPosition __attribute__((deprecated))
#else
# define DEPRECATED__messages__msg__ZedPosition __declspec(deprecated)
#endif

namespace messages
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ZedPosition_
{
  using Type = ZedPosition_<ContainerAllocator>;

  explicit ZedPosition_(rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  {
    if (rosidl_generator_cpp::MessageInitialization::ALL == _init ||
      rosidl_generator_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->ax = 0.0f;
      this->ay = 0.0f;
      this->az = 0.0f;
    }
  }

  explicit ZedPosition_(const ContainerAllocator & _alloc, rosidl_generator_cpp::MessageInitialization _init = rosidl_generator_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_generator_cpp::MessageInitialization::ALL == _init ||
      rosidl_generator_cpp::MessageInitialization::ZERO == _init)
    {
      this->x = 0.0f;
      this->y = 0.0f;
      this->z = 0.0f;
      this->ax = 0.0f;
      this->ay = 0.0f;
      this->az = 0.0f;
    }
  }

  // field types and members
  using _x_type =
    float;
  _x_type x;
  using _y_type =
    float;
  _y_type y;
  using _z_type =
    float;
  _z_type z;
  using _ax_type =
    float;
  _ax_type ax;
  using _ay_type =
    float;
  _ay_type ay;
  using _az_type =
    float;
  _az_type az;

  // setters for named parameter idiom
  Type & set__x(
    const float & _arg)
  {
    this->x = _arg;
    return *this;
  }
  Type & set__y(
    const float & _arg)
  {
    this->y = _arg;
    return *this;
  }
  Type & set__z(
    const float & _arg)
  {
    this->z = _arg;
    return *this;
  }
  Type & set__ax(
    const float & _arg)
  {
    this->ax = _arg;
    return *this;
  }
  Type & set__ay(
    const float & _arg)
  {
    this->ay = _arg;
    return *this;
  }
  Type & set__az(
    const float & _arg)
  {
    this->az = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    messages::msg::ZedPosition_<ContainerAllocator> *;
  using ConstRawPtr =
    const messages::msg::ZedPosition_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<messages::msg::ZedPosition_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<messages::msg::ZedPosition_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      messages::msg::ZedPosition_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<messages::msg::ZedPosition_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      messages::msg::ZedPosition_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<messages::msg::ZedPosition_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<messages::msg::ZedPosition_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<messages::msg::ZedPosition_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__messages__msg__ZedPosition
    std::shared_ptr<messages::msg::ZedPosition_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__messages__msg__ZedPosition
    std::shared_ptr<messages::msg::ZedPosition_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ZedPosition_ & other) const
  {
    if (this->x != other.x) {
      return false;
    }
    if (this->y != other.y) {
      return false;
    }
    if (this->z != other.z) {
      return false;
    }
    if (this->ax != other.ax) {
      return false;
    }
    if (this->ay != other.ay) {
      return false;
    }
    if (this->az != other.az) {
      return false;
    }
    return true;
  }
  bool operator!=(const ZedPosition_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ZedPosition_

// alias to use template instance with default allocator
using ZedPosition =
  messages::msg::ZedPosition_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace messages

#endif  // MESSAGES__MSG__ZED_POSITION__STRUCT_HPP_
