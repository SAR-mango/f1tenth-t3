// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from drive_msgs:msg/DriveParam.idl
// generated code does not contain a copyright notice

#ifndef DRIVE_MSGS__MSG__DETAIL__DRIVE_PARAM__STRUCT_H_
#define DRIVE_MSGS__MSG__DETAIL__DRIVE_PARAM__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/DriveParam in the package drive_msgs.
typedef struct drive_msgs__msg__DriveParam
{
  float velocity;
  float angle;
} drive_msgs__msg__DriveParam;

// Struct for a sequence of drive_msgs__msg__DriveParam.
typedef struct drive_msgs__msg__DriveParam__Sequence
{
  drive_msgs__msg__DriveParam * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} drive_msgs__msg__DriveParam__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DRIVE_MSGS__MSG__DETAIL__DRIVE_PARAM__STRUCT_H_
