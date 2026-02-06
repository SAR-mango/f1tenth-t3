// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from drive_msgs:msg/DriveParam.idl
// generated code does not contain a copyright notice

#ifndef DRIVE_MSGS__MSG__DETAIL__DRIVE_PARAM__STRUCT_HPP_
#define DRIVE_MSGS__MSG__DETAIL__DRIVE_PARAM__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__drive_msgs__msg__DriveParam __attribute__((deprecated))
#else
# define DEPRECATED__drive_msgs__msg__DriveParam __declspec(deprecated)
#endif

namespace drive_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct DriveParam_
{
  using Type = DriveParam_<ContainerAllocator>;

  explicit DriveParam_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->velocity = 0.0f;
      this->angle = 0.0f;
    }
  }

  explicit DriveParam_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->velocity = 0.0f;
      this->angle = 0.0f;
    }
  }

  // field types and members
  using _velocity_type =
    float;
  _velocity_type velocity;
  using _angle_type =
    float;
  _angle_type angle;

  // setters for named parameter idiom
  Type & set__velocity(
    const float & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__angle(
    const float & _arg)
  {
    this->angle = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    drive_msgs::msg::DriveParam_<ContainerAllocator> *;
  using ConstRawPtr =
    const drive_msgs::msg::DriveParam_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<drive_msgs::msg::DriveParam_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<drive_msgs::msg::DriveParam_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      drive_msgs::msg::DriveParam_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<drive_msgs::msg::DriveParam_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      drive_msgs::msg::DriveParam_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<drive_msgs::msg::DriveParam_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<drive_msgs::msg::DriveParam_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<drive_msgs::msg::DriveParam_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__drive_msgs__msg__DriveParam
    std::shared_ptr<drive_msgs::msg::DriveParam_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__drive_msgs__msg__DriveParam
    std::shared_ptr<drive_msgs::msg::DriveParam_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const DriveParam_ & other) const
  {
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->angle != other.angle) {
      return false;
    }
    return true;
  }
  bool operator!=(const DriveParam_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct DriveParam_

// alias to use template instance with default allocator
using DriveParam =
  drive_msgs::msg::DriveParam_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace drive_msgs

#endif  // DRIVE_MSGS__MSG__DETAIL__DRIVE_PARAM__STRUCT_HPP_
