#include "car_control/drive_parameters_multiplexer.hpp"

DriveParametersMultiplexer::DriveParametersMultiplexer()
    : Node("drive_parameters_multiplexer")
    , last_updated_source_(nullptr)
    , drive_mode_(DriveMode::LOCKED)
{
    drive_parameters_publisher_ =
        this->create_publisher<drive_msgs::msg::DriveParam>(TOPIC_DRIVE_PARAM, rclcpp::QoS(1));

    auto callback =
        std::bind(&DriveParametersMultiplexer::on_update, this, std::placeholders::_1, std::placeholders::_2);

    sources_ = {
        std::make_unique<DriveParametersSource>(
            this,
            TOPIC_DRIVE_PARAMETERS_KEYBOARD,
            callback,
            DriveMode::MANUAL,
            rclcpp::Duration::from_seconds(0.1)),
        std::make_unique<DriveParametersSource>(
            this,
            TOPIC_DRIVE_PARAMETERS_JOYSTICK,
            callback,
            DriveMode::MANUAL,
            rclcpp::Duration::from_seconds(0.1)),
        std::make_unique<DriveParametersSource>(
            this,
            TOPIC_DRIVE_PARAMETERS_AUTONOMOUS,
            callback,
            DriveMode::AUTONOMOUS,
            rclcpp::Duration::from_seconds(0.1)),
    };

    drive_mode_subscriber_ = this->create_subscription<std_msgs::msg::Int32>(
        TOPIC_DRIVE_MODE,
        rclcpp::QoS(1),
        std::bind(&DriveParametersMultiplexer::drive_mode_callback, this, std::placeholders::_1));
}

bool DriveParametersMultiplexer::validate_source(DriveParametersSource* source) const
{
    if (source == nullptr)
    {
        return false;
    }

    if (source->get_drive_mode() != drive_mode_)
    {
        return false;
    }

    return last_updated_source_ == nullptr || last_updated_source_ == source
        || last_updated_source_->is_outdated()
        || last_updated_source_->get_drive_mode() != drive_mode_
        || (!source->is_idle() && last_updated_source_->is_idle());
}

void DriveParametersMultiplexer::on_update(
    DriveParametersSource* source,
    const drive_msgs::msg::DriveParam::SharedPtr& message)
{
    if (!validate_source(source))
    {
        return;
    }
    drive_parameters_publisher_->publish(*message);
    last_updated_source_ = source;
}

void DriveParametersMultiplexer::drive_mode_callback(const std_msgs::msg::Int32::SharedPtr message)
{
    drive_mode_ = static_cast<DriveMode>(message->data);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DriveParametersMultiplexer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
