#include "car_control/car_controller.hpp"

#include "car_control/car_config.hpp"

CarController::CarController()
    : Node("car_controller")
    , drive_param_lock_(true)
    , emergency_stop_lock_(true)
    , current_drive_mode_(DriveMode::LOCKED)
    , publish_cmd_vel_(true)
    , max_linear_speed_(1.0)
    , max_steering_angle_(0.5)
{
    publish_cmd_vel_ = this->declare_parameter<bool>("publish_cmd_vel", true);
    max_linear_speed_ = this->declare_parameter<double>("max_linear_speed", 1.0);
    max_steering_angle_ = this->declare_parameter<double>("max_steering_angle", 0.5);

    drive_parameters_subscriber_ = this->create_subscription<drive_msgs::msg::DriveParam>(
        TOPIC_DRIVE_PARAM,
        rclcpp::QoS(1),
        std::bind(&CarController::drive_parameters_callback, this, std::placeholders::_1));
    drive_mode_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
        TOPIC_DRIVE_MODE,
        rclcpp::QoS(1),
        std::bind(&CarController::drive_mode_callback, this, std::placeholders::_1));
    emergency_stop_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        TOPIC_EMERGENCY_STOP,
        rclcpp::QoS(1),
        std::bind(&CarController::emergency_stop_callback, this, std::placeholders::_1));

    speed_publisher_ = this->create_publisher<std_msgs::msg::Float64>(TOPIC_FOCBOX_SPEED, rclcpp::QoS(1));
    angle_publisher_ = this->create_publisher<std_msgs::msg::Float64>(TOPIC_FOCBOX_ANGLE, rclcpp::QoS(1));
    brake_publisher_ = this->create_publisher<std_msgs::msg::Float64>(TOPIC_FOCBOX_BRAKE, rclcpp::QoS(1));
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(TOPIC_CMD_VEL, rclcpp::QoS(1));
}

void CarController::drive_parameters_callback(const drive_msgs::msg::DriveParam::SharedPtr parameters)
{
    const double relative_speed = (drive_param_lock_ || emergency_stop_lock_) ? 0.0 : parameters->velocity;
    const double relative_angle = drive_param_lock_ ? 0.0 : parameters->angle;
    publish_drive_parameters(relative_speed, relative_angle);
    if (publish_cmd_vel_)
    {
        publish_cmd_vel(relative_speed, relative_angle);
    }
}

void CarController::publish_drive_parameters(double relative_speed, double relative_angle)
{
    const double speed = relative_speed * car_config::MAX_RPM_ELECTRICAL;
    const double angle = (relative_angle * car_config::MAX_SERVO_POSITION + car_config::MAX_SERVO_POSITION) / 2.0;

    publish_speed(speed);
    publish_angle(angle);
}

void CarController::publish_speed(double speed)
{
    std_msgs::msg::Float64 speed_message;
    speed_message.data = speed;
    speed_publisher_->publish(speed_message);
}

void CarController::publish_angle(double angle)
{
    std_msgs::msg::Float64 angle_message;
    angle_message.data = angle;
    angle_publisher_->publish(angle_message);
}

void CarController::publish_cmd_vel(double relative_speed, double relative_angle)
{
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = relative_speed * max_linear_speed_;
    cmd.angular.z = relative_angle * max_steering_angle_;
    cmd_vel_publisher_->publish(cmd);
}

void CarController::drive_mode_callback(const std_msgs::msg::Int32::SharedPtr drive_mode_message)
{
    current_drive_mode_ = static_cast<DriveMode>(drive_mode_message->data);
    drive_param_lock_ = current_drive_mode_ == DriveMode::LOCKED;
    if (drive_param_lock_)
    {
        stop();
    }
}

void CarController::emergency_stop_callback(const std_msgs::msg::Bool::SharedPtr emergency_stop_message)
{
    const bool enable_emergency_stop = emergency_stop_message->data && current_drive_mode_ != DriveMode::MANUAL;
    emergency_stop_lock_ = enable_emergency_stop;
    if (emergency_stop_lock_)
    {
        stop();
    }
}

void CarController::stop()
{
    publish_speed(0.0);

    std_msgs::msg::Float64 brake_message;
    brake_message.data = 0.0;
    brake_publisher_->publish(brake_message);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CarController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
