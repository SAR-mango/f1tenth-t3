#pragma once

#include "car_control/drive_mode.hpp"
#include <drive_msgs/msg/drive_param.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>

constexpr const char* TOPIC_FOCBOX_SPEED = "/commands/motor/speed";
constexpr const char* TOPIC_FOCBOX_ANGLE = "/commands/servo/position";
constexpr const char* TOPIC_FOCBOX_BRAKE = "commands/motor/brake";
constexpr const char* TOPIC_DRIVE_PARAM = "/commands/drive_param";
constexpr const char* TOPIC_DRIVE_MODE = "/commands/drive_mode";
constexpr const char* TOPIC_EMERGENCY_STOP = "/commands/emergency_stop";
constexpr const char* TOPIC_CMD_VEL = "/cmd_vel";

class CarController : public rclcpp::Node
{
public:
    CarController();

private:
    void drive_parameters_callback(const drive_msgs::msg::DriveParam::SharedPtr parameters);
    void drive_mode_callback(const std_msgs::msg::Int32::SharedPtr drive_mode_message);
    void emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr emergency_stop_message);

    void publish_drive_parameters(double relative_speed, double relative_angle);
    void publish_speed(double speed);
    void publish_angle(double angle);
    void publish_cmd_vel(double relative_speed, double relative_angle);
    void stop();

    rclcpp::Subscription<drive_msgs::msg::DriveParam>::SharedPtr drive_parameters_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr drive_mode_subscriber_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_subscriber_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speed_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr brake_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    bool drive_param_lock_;
    bool emergency_stop_lock_;
    DriveMode current_drive_mode_;

    bool publish_cmd_vel_;
    double max_linear_speed_;
    double max_steering_angle_;
};
