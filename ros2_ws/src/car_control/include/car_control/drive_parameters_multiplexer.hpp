#pragma once

#include <array>
#include <memory>

#include "car_control/drive_mode.hpp"
#include "car_control/drive_parameters_source.hpp"
#include <drive_msgs/msg/drive_param.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

constexpr const char* TOPIC_DRIVE_PARAM = "/commands/drive_param";
constexpr const char* TOPIC_DRIVE_PARAMETERS_KEYBOARD = "input/drive_param/keyboard";
constexpr const char* TOPIC_DRIVE_PARAMETERS_JOYSTICK = "input/drive_param/joystick";
constexpr const char* TOPIC_DRIVE_PARAMETERS_AUTONOMOUS = "input/drive_param/autonomous";
constexpr const char* TOPIC_DRIVE_MODE = "/commands/drive_mode";

class DriveParametersMultiplexer : public rclcpp::Node
{
public:
    DriveParametersMultiplexer();

private:
    bool validate_source(DriveParametersSource* source) const;
    void on_update(
        DriveParametersSource* source,
        const drive_msgs::msg::DriveParam::SharedPtr& message);
    void drive_mode_callback(const std_msgs::msg::Int32::SharedPtr message);

    std::array<std::unique_ptr<DriveParametersSource>, 3> sources_;
    DriveParametersSource* last_updated_source_;
    rclcpp::Publisher<drive_msgs::msg::DriveParam>::SharedPtr drive_parameters_publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr drive_mode_subscriber_;

    DriveMode drive_mode_;
};
