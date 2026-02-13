// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from drive_msgs:msg/DriveParam.idl
// generated code does not contain a copyright notice

#ifndef DRIVE_MSGS__MSG__DETAIL__DRIVE_PARAM__TRAITS_HPP_
#define DRIVE_MSGS__MSG__DETAIL__DRIVE_PARAM__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "drive_msgs/msg/detail/drive_param__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace drive_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const DriveParam & msg,
  std::ostream & out)
{
  out << "{";
  // member: velocity
  {
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << ", ";
  }

  // member: angle
  {
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const DriveParam & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: velocity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "velocity: ";
    rosidl_generator_traits::value_to_yaml(msg.velocity, out);
    out << "\n";
  }

  // member: angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "angle: ";
    rosidl_generator_traits::value_to_yaml(msg.angle, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const DriveParam & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace drive_msgs

namespace rosidl_generator_traits
{

[[deprecated("use drive_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const drive_msgs::msg::DriveParam & msg,
  std::ostream & out, size_t indentation = 0)
{
  drive_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use drive_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const drive_msgs::msg::DriveParam & msg)
{
  return drive_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<drive_msgs::msg::DriveParam>()
{
  return "drive_msgs::msg::DriveParam";
}

template<>
inline const char * name<drive_msgs::msg::DriveParam>()
{
  return "drive_msgs/msg/DriveParam";
}

template<>
struct has_fixed_size<drive_msgs::msg::DriveParam>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<drive_msgs::msg::DriveParam>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<drive_msgs::msg::DriveParam>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DRIVE_MSGS__MSG__DETAIL__DRIVE_PARAM__TRAITS_HPP_
