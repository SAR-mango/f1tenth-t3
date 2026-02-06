// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from drive_msgs:msg/DriveParam.idl
// generated code does not contain a copyright notice

#ifndef DRIVE_MSGS__MSG__DETAIL__DRIVE_PARAM__BUILDER_HPP_
#define DRIVE_MSGS__MSG__DETAIL__DRIVE_PARAM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "drive_msgs/msg/detail/drive_param__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace drive_msgs
{

namespace msg
{

namespace builder
{

class Init_DriveParam_angle
{
public:
  explicit Init_DriveParam_angle(::drive_msgs::msg::DriveParam & msg)
  : msg_(msg)
  {}
  ::drive_msgs::msg::DriveParam angle(::drive_msgs::msg::DriveParam::_angle_type arg)
  {
    msg_.angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::drive_msgs::msg::DriveParam msg_;
};

class Init_DriveParam_velocity
{
public:
  Init_DriveParam_velocity()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DriveParam_angle velocity(::drive_msgs::msg::DriveParam::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_DriveParam_angle(msg_);
  }

private:
  ::drive_msgs::msg::DriveParam msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::drive_msgs::msg::DriveParam>()
{
  return drive_msgs::msg::builder::Init_DriveParam_velocity();
}

}  // namespace drive_msgs

#endif  // DRIVE_MSGS__MSG__DETAIL__DRIVE_PARAM__BUILDER_HPP_
