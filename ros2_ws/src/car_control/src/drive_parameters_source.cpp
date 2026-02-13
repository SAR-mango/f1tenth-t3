#include "car_control/drive_parameters_source.hpp"

#include <cmath>
#include <stdexcept>

DriveParametersSource::DriveParametersSource(
    rclcpp::Node* node,
    const std::string& topic,
    DriveParameterCallbackFunction update_callback,
    DriveMode drive_mode,
    rclcpp::Duration timeout)
    : node_(node)
    , drive_mode_(drive_mode)
    , idle_(true)
    , has_update_(false)
    , timeout_(timeout)
    , update_callback_(update_callback)
{
    if (drive_mode == DriveMode::LOCKED)
    {
        throw std::runtime_error("LOCKED is not a valid drive parameter source.");
    }

    drive_parameters_subscriber_ = node_->create_subscription<drive_msgs::msg::DriveParam>(
        topic,
        rclcpp::QoS(1),
        [this](const drive_msgs::msg::DriveParam::SharedPtr message) {
            drive_parameters_callback(message);
        });
}

void DriveParametersSource::drive_parameters_callback(const drive_msgs::msg::DriveParam::SharedPtr message)
{
    last_update_ = node_->now();
    has_update_ = true;
    idle_ = std::abs(message->velocity) < IDLE_RANGE && std::abs(message->angle) < IDLE_RANGE;
    update_callback_(this, message);
}

bool DriveParametersSource::is_outdated() const
{
    if (!has_update_)
    {
        return true;
    }
    return last_update_ + timeout_ < node_->now();
}

bool DriveParametersSource::is_idle() const
{
    return idle_;
}

DriveMode DriveParametersSource::get_drive_mode() const
{
    return drive_mode_;
}
