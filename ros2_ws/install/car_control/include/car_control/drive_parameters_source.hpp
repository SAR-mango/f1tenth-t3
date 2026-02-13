#pragma once

#include <functional>
#include <memory>
#include <string>

#include "car_control/drive_mode.hpp"
#include <drive_msgs/msg/drive_param.hpp>
#include <rclcpp/rclcpp.hpp>

class DriveParametersSource;
using DriveParameterCallbackFunction = std::function<void(
    DriveParametersSource*, drive_msgs::msg::DriveParam::SharedPtr)>;

constexpr float IDLE_RANGE = 0.01f;

class DriveParametersSource
{
public:
    DriveParametersSource(
        rclcpp::Node* node,
        const std::string& topic,
        DriveParameterCallbackFunction update_callback,
        DriveMode drive_mode,
        rclcpp::Duration timeout);

    bool is_outdated() const;
    bool is_idle() const;
    DriveMode get_drive_mode() const;

private:
    void drive_parameters_callback(const drive_msgs::msg::DriveParam::SharedPtr message);

    rclcpp::Node* node_;
    rclcpp::Subscription<drive_msgs::msg::DriveParam>::SharedPtr drive_parameters_subscriber_;

    DriveMode drive_mode_;
    bool idle_;
    bool has_update_;
    rclcpp::Duration timeout_;
    rclcpp::Time last_update_;

    DriveParameterCallbackFunction update_callback_;
};
