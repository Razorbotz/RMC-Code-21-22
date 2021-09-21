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
      this->ox = 0.0f;
      this->oy = 0.0f;
      this->oz = 0.0f;
      this->ow = 0.0f;
      this->aruco_visible = false;
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
      this->ox = 0.0f;
      this->oy = 0.0f;
      this->oz = 0.0f;
      this->ow = 0.0f;
      this->aruco_visible = false;
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
  using _ox_type =
    float;
  _ox_type ox;
  using _oy_type =
    float;
  _oy_type oy;
  using _oz_type =
    float;
  _oz_type oz;
  using _ow_type =
    float;
  _ow_type ow;
  using _aruco_visible_type =
    bool;
  _aruco_visible_type aruco_visible;

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
  Type & set__ox(
    const float & _arg)
  {
    this->ox = _arg;
    return *this;
  }
  Type & set__oy(
    const float & _arg)
  {
    this->oy = _arg;
    return *this;
  }
  Type & set__oz(
    const float & _arg)
  {
    this->oz = _arg;
    return *this;
  }
  Type & set__ow(
    const float & _arg)
  {
    this->ow = _arg;
    return *this;
  }
  Type & set__aruco_visible(
    const bool & _arg)
  {
    this->aruco_visible = _arg;
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
    if (this->ox != other.ox) {
      return false;
    }
    if (this->oy != other.oy) {
      return false;
    }
    if (this->oz != other.oz) {
      return false;
    }
    if (this->ow != other.ow) {
      return false;
    }
    if (this->aruco_visible != other.aruco_visible) {
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
