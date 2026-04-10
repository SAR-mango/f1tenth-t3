#include "car_control/car_controller.hpp"

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <thread>

namespace
{
speed_t baud_rate_to_constant(int baud_rate)
{
    switch (baud_rate)
    {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
#ifdef B460800
        case 460800:
            return B460800;
#endif
#ifdef B500000
        case 500000:
            return B500000;
#endif
#ifdef B576000
        case 576000:
            return B576000;
#endif
#ifdef B921600
        case 921600:
            return B921600;
#endif
#ifdef B1000000
        case 1000000:
            return B1000000;
#endif
        default:
            return B0;
    }
}

std::string decode_escapes(const std::string& input)
{
    std::string result;
    result.reserve(input.size());

    for (std::size_t i = 0; i < input.size(); ++i)
    {
        if (input[i] != '\\' || i + 1 >= input.size())
        {
            result.push_back(input[i]);
            continue;
        }

        ++i;
        switch (input[i])
        {
            case 'n':
                result.push_back('\n');
                break;
            case 'r':
                result.push_back('\r');
                break;
            case 't':
                result.push_back('\t');
                break;
            case '\\':
                result.push_back('\\');
                break;
            default:
                result.push_back(input[i]);
                break;
        }
    }

    return result;
}
} // namespace

class UartActuatorBridge : public rclcpp::Node
{
public:
    UartActuatorBridge()
        : Node("uart_actuator_bridge")
        , command_mode_("cmd_vel")
        , include_brake_(true)
        , decimal_places_(6)
        , command_timeout_sec_(0.5)
        , reconnect_interval_sec_(1.0)
        , fd_(-1)
        , speed_(0.0)
        , angle_(0.0)
        , brake_(0.0)
        , last_open_attempt_(std::chrono::steady_clock::time_point{})
        , last_command_time_(std::chrono::steady_clock::now())
    {
        command_mode_ = this->declare_parameter<std::string>("command_mode", "cmd_vel");
        if (command_mode_ != "cmd_vel" && command_mode_ != "actuator_topics")
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Unsupported command_mode '%s'. Falling back to cmd_vel.",
                command_mode_.c_str());
            command_mode_ = "cmd_vel";
        }

        uart_device_ = this->declare_parameter<std::string>("uart_device", "/dev/ttyTHS1");
        baud_rate_ = static_cast<int>(this->declare_parameter<int64_t>("baud_rate", 115200));
        cmd_vel_topic_ = this->declare_parameter<std::string>("cmd_vel_topic", TOPIC_CMD_VEL);
        speed_topic_ = this->declare_parameter<std::string>("speed_topic", TOPIC_FOCBOX_SPEED);
        angle_topic_ = this->declare_parameter<std::string>("angle_topic", TOPIC_FOCBOX_ANGLE);
        brake_topic_ = this->declare_parameter<std::string>("brake_topic", TOPIC_FOCBOX_BRAKE);
        frame_prefix_ = this->declare_parameter<std::string>("frame_prefix", "");
        line_terminator_ = decode_escapes(this->declare_parameter<std::string>("line_terminator", "\\n"));
        include_brake_ =
            this->declare_parameter<bool>("include_brake", command_mode_ == "actuator_topics");
        const auto declared_decimal_places = this->declare_parameter<int64_t>("decimal_places", 6);
        decimal_places_ = static_cast<int>(std::max<int64_t>(0, declared_decimal_places));
        command_timeout_sec_ = std::max(0.0, this->declare_parameter<double>("command_timeout_sec", 0.5));
        reconnect_interval_sec_ =
            std::max(0.1, this->declare_parameter<double>("reconnect_interval_sec", 1.0));
        const double send_rate_hz = std::max(1.0, this->declare_parameter<double>("send_rate_hz", 40.0));

        if (command_mode_ == "cmd_vel")
        {
            cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                cmd_vel_topic_,
                rclcpp::QoS(10),
                std::bind(&UartActuatorBridge::cmd_vel_callback, this, std::placeholders::_1));
        }
        else
        {
            speed_sub_ = this->create_subscription<std_msgs::msg::Float64>(
                speed_topic_,
                rclcpp::QoS(10),
                std::bind(&UartActuatorBridge::speed_callback, this, std::placeholders::_1));
            angle_sub_ = this->create_subscription<std_msgs::msg::Float64>(
                angle_topic_,
                rclcpp::QoS(10),
                std::bind(&UartActuatorBridge::angle_callback, this, std::placeholders::_1));
            brake_sub_ = this->create_subscription<std_msgs::msg::Float64>(
                brake_topic_,
                rclcpp::QoS(10),
                std::bind(&UartActuatorBridge::brake_callback, this, std::placeholders::_1));
        }

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / send_rate_hz),
            std::bind(&UartActuatorBridge::timer_callback, this));

        if (command_mode_ == "cmd_vel")
        {
            RCLCPP_INFO(
                this->get_logger(),
                "UART bridge ready. mode=cmd_vel device=%s baud=%d cmd_vel_topic=%s include_brake=%s",
                uart_device_.c_str(),
                baud_rate_,
                cmd_vel_topic_.c_str(),
                include_brake_ ? "true" : "false");
        }
        else
        {
            RCLCPP_INFO(
                this->get_logger(),
                "UART bridge ready. mode=actuator_topics device=%s baud=%d speed_topic=%s "
                "angle_topic=%s brake_topic=%s include_brake=%s",
                uart_device_.c_str(),
                baud_rate_,
                speed_topic_.c_str(),
                angle_topic_.c_str(),
                brake_topic_.c_str(),
                include_brake_ ? "true" : "false");
        }
    }

    ~UartActuatorBridge() override
    {
        send_stop_frame_on_shutdown();
        close_port();
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr message)
    {
        speed_ = message->linear.x;
        angle_ = message->angular.z;
        brake_ = 0.0;
        last_command_time_ = std::chrono::steady_clock::now();
    }

    void speed_callback(const std_msgs::msg::Float64::SharedPtr message)
    {
        speed_ = message->data;
        last_command_time_ = std::chrono::steady_clock::now();
    }

    void angle_callback(const std_msgs::msg::Float64::SharedPtr message)
    {
        angle_ = message->data;
        last_command_time_ = std::chrono::steady_clock::now();
    }

    void brake_callback(const std_msgs::msg::Float64::SharedPtr message)
    {
        brake_ = message->data;
        last_command_time_ = std::chrono::steady_clock::now();
    }

    void timer_callback()
    {
        if (fd_ < 0)
        {
            maybe_open_port();
            return;
        }

        const auto now = std::chrono::steady_clock::now();
        double speed = speed_;
        double angle = angle_;
        double brake = brake_;

        if (command_timeout_sec_ > 0.0)
        {
            const std::chrono::duration<double> age = now - last_command_time_;
            if (age.count() > command_timeout_sec_)
            {
                speed = 0.0;
                angle = 0.0;
                brake = 0.0;
            }
        }

        const std::string frame = build_frame(speed, angle, brake);
        if (!write_all(frame))
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "UART write failed on %s: %s. Closing port and retrying.",
                uart_device_.c_str(),
                strerror(errno));
            close_port();
        }
    }

    void maybe_open_port()
    {
        const auto now = std::chrono::steady_clock::now();
        const std::chrono::duration<double> since_last_attempt = now - last_open_attempt_;
        if (since_last_attempt.count() < reconnect_interval_sec_)
        {
            return;
        }
        last_open_attempt_ = now;

        fd_ = ::open(uart_device_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0)
        {
            RCLCPP_WARN(
                this->get_logger(),
                "Failed to open UART device %s: %s",
                uart_device_.c_str(),
                strerror(errno));
            return;
        }

        if (!configure_port())
        {
            RCLCPP_ERROR(
                this->get_logger(),
                "Failed to configure UART device %s at %d baud.",
                uart_device_.c_str(),
                baud_rate_);
            close_port();
            return;
        }

        RCLCPP_INFO(
            this->get_logger(),
            "Opened UART device %s at %d baud.",
            uart_device_.c_str(),
            baud_rate_);
    }

    bool configure_port()
    {
        const speed_t baud_constant = baud_rate_to_constant(baud_rate_);
        if (baud_constant == B0)
        {
            RCLCPP_ERROR(this->get_logger(), "Unsupported UART baud rate: %d", baud_rate_);
            return false;
        }

        termios tty {};
        if (tcgetattr(fd_, &tty) != 0)
        {
            return false;
        }

        cfmakeraw(&tty);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
#ifdef CRTSCTS
        tty.c_cflag &= ~CRTSCTS;
#endif
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        if (cfsetispeed(&tty, baud_constant) != 0 || cfsetospeed(&tty, baud_constant) != 0)
        {
            return false;
        }

        if (tcsetattr(fd_, TCSANOW, &tty) != 0)
        {
            return false;
        }

        return true;
    }

    void close_port()
    {
        if (fd_ >= 0)
        {
            ::close(fd_);
            fd_ = -1;
        }
    }

    void send_stop_frame_on_shutdown()
    {
        if (fd_ < 0)
        {
            return;
        }

        const std::string stop_frame = build_frame(0.0, 0.0, 0.0);
        constexpr int kShutdownFrameRepeatCount = 3;
        constexpr auto kShutdownFrameRepeatDelay = std::chrono::milliseconds(20);

        for (int i = 0; i < kShutdownFrameRepeatCount; ++i)
        {
            if (!write_all(stop_frame) || !drain_output())
            {
                break;
            }

            if (i + 1 < kShutdownFrameRepeatCount)
            {
                std::this_thread::sleep_for(kShutdownFrameRepeatDelay);
            }
        }
    }

    std::string build_frame(double speed, double angle, double brake) const
    {
        std::ostringstream stream;
        stream << frame_prefix_ << std::fixed << std::setprecision(decimal_places_) << speed << "," << angle;
        if (include_brake_)
        {
            stream << "," << brake;
        }
        stream << line_terminator_;
        return stream.str();
    }

    bool write_all(const std::string& payload)
    {
        std::size_t written_total = 0;
        while (written_total < payload.size())
        {
            const ssize_t written = ::write(
                fd_,
                payload.data() + written_total,
                payload.size() - written_total);
            if (written < 0)
            {
                if (errno == EINTR)
                {
                    continue;
                }
                return false;
            }
            if (written == 0)
            {
                return false;
            }
            written_total += static_cast<std::size_t>(written);
        }
        return true;
    }

    bool drain_output()
    {
        while (tcdrain(fd_) != 0)
        {
            if (errno == EINTR)
            {
                continue;
            }
            return false;
        }
        return true;
    }

    std::string command_mode_;
    std::string uart_device_;
    int baud_rate_;
    std::string cmd_vel_topic_;
    std::string speed_topic_;
    std::string angle_topic_;
    std::string brake_topic_;
    std::string frame_prefix_;
    std::string line_terminator_;
    bool include_brake_;
    int decimal_places_;
    double command_timeout_sec_;
    double reconnect_interval_sec_;

    int fd_;
    double speed_;
    double angle_;
    double brake_;

    std::chrono::steady_clock::time_point last_open_attempt_;
    std::chrono::steady_clock::time_point last_command_time_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr speed_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr brake_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UartActuatorBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
