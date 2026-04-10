import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _car_control_exec(name: str) -> str:
    repo_root = os.environ.get("F1TENTH_T3_ROOT", os.path.expanduser("~/f1tenth-t3"))
    return os.path.join(
        repo_root,
        "ros2_ws",
        "install",
        "car_control",
        "lib",
        "car_control",
        name,
    )


def generate_launch_description():
    start_lidar_driver = LaunchConfiguration("start_lidar_driver")
    start_uart_bridge = LaunchConfiguration("start_uart_bridge")
    lidar_ip = LaunchConfiguration("lidar_ip")
    lidar_port = LaunchConfiguration("lidar_port")
    scan_topic = LaunchConfiguration("scan_topic")
    laser_frame_id = LaunchConfiguration("laser_frame_id")

    uart_command_mode = LaunchConfiguration("uart_command_mode")
    uart_cmd_vel_topic = LaunchConfiguration("uart_cmd_vel_topic")
    uart_device = LaunchConfiguration("uart_device")
    uart_baud_rate = LaunchConfiguration("uart_baud_rate")
    uart_send_rate_hz = LaunchConfiguration("uart_send_rate_hz")
    uart_command_timeout_sec = LaunchConfiguration("uart_command_timeout_sec")

    car_max_linear_speed = LaunchConfiguration("car_max_linear_speed")
    car_max_steering_angle = LaunchConfiguration("car_max_steering_angle")

    start_lidar_driver_arg = DeclareLaunchArgument(
        "start_lidar_driver",
        default_value="true",
        description="If true, launch urg_node_driver for Hokuyo over Ethernet.",
    )
    start_uart_bridge_arg = DeclareLaunchArgument(
        "start_uart_bridge",
        default_value="true",
        description="If true, launch the UART actuator bridge for Jetson serial output.",
    )
    lidar_ip_arg = DeclareLaunchArgument(
        "lidar_ip",
        default_value="192.168.0.10",
        description="Hokuyo UST-10LX IPv4 address.",
    )
    lidar_port_arg = DeclareLaunchArgument(
        "lidar_port",
        default_value="10940",
        description="Hokuyo UST-10LX TCP port.",
    )
    scan_topic_arg = DeclareLaunchArgument(
        "scan_topic",
        default_value="/scan",
        description="LaserScan topic used by the wallbalancing node.",
    )
    laser_frame_id_arg = DeclareLaunchArgument(
        "laser_frame_id",
        default_value="car/chassis/ust10lx",
        description="Frame id for published LaserScan messages.",
    )

    uart_command_mode_arg = DeclareLaunchArgument(
        "uart_command_mode",
        default_value="cmd_vel",
        description="UART bridge input mode: cmd_vel or actuator_topics.",
    )
    uart_cmd_vel_topic_arg = DeclareLaunchArgument(
        "uart_cmd_vel_topic",
        default_value="/cmd_vel",
        description="Twist topic forwarded over UART when using cmd_vel mode.",
    )
    uart_device_arg = DeclareLaunchArgument(
        "uart_device",
        default_value="/dev/ttyTHS1",
        description="Jetson UART device used for actuator output.",
    )
    uart_baud_rate_arg = DeclareLaunchArgument(
        "uart_baud_rate",
        default_value="115200",
        description="UART baud rate for actuator output.",
    )
    uart_send_rate_arg = DeclareLaunchArgument(
        "uart_send_rate_hz",
        default_value="40.0",
        description="How often the UART bridge transmits the latest command frame.",
    )
    uart_command_timeout_arg = DeclareLaunchArgument(
        "uart_command_timeout_sec",
        default_value="0.5",
        description="If no new command arrives within this timeout, the UART bridge sends zeros.",
    )

    car_max_linear_speed_arg = DeclareLaunchArgument(
        "car_max_linear_speed",
        default_value="1.0",
        description="Maximum linear speed in m/s that car_controller maps from full throttle.",
    )
    car_max_steering_angle_arg = DeclareLaunchArgument(
        "car_max_steering_angle",
        default_value="0.54",
        description="Maximum steering angle in rad that car_controller maps from full steering command.",
    )

    urg_driver = Node(
        package="urg_node",
        executable="urg_node_driver",
        output="screen",
        condition=IfCondition(start_lidar_driver),
        parameters=[
            {
                "ip_address": lidar_ip,
                "ip_port": lidar_port,
                "laser_frame_id": laser_frame_id,
                "angle_min": -2.35619449,
                "angle_max": 2.35619449,
            }
        ],
        remappings=[
            ("/scan", scan_topic),
        ],
    )

    drive_parameters_multiplexer = ExecuteProcess(
        cmd=[
            _car_control_exec("drive_parameters_multiplexer"),
        ],
        output="screen",
    )

    car_controller = ExecuteProcess(
        cmd=[
            _car_control_exec("car_controller"),
            "--ros-args",
            "-p",
            "publish_cmd_vel:=true",
            "-p",
            ["max_linear_speed:=", car_max_linear_speed],
            "-p",
            ["max_steering_angle:=", car_max_steering_angle],
            "-r",
            ["/cmd_vel:=", uart_cmd_vel_topic],
        ],
        output="screen",
    )

    uart_actuator_bridge = ExecuteProcess(
        cmd=[
            _car_control_exec("uart_actuator_bridge"),
            "--ros-args",
            "-p",
            ["command_mode:=", uart_command_mode],
            "-p",
            ["cmd_vel_topic:=", uart_cmd_vel_topic],
            "-p",
            ["uart_device:=", uart_device],
            "-p",
            ["baud_rate:=", uart_baud_rate],
            "-p",
            ["send_rate_hz:=", uart_send_rate_hz],
            "-p",
            ["command_timeout_sec:=", uart_command_timeout_sec],
        ],
        output="screen",
        condition=IfCondition(start_uart_bridge),
    )

    drive_mode_pub = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "-r",
            "2",
            "/commands/drive_mode",
            "std_msgs/msg/Int32",
            "{data: 2}",
        ],
        output="log",
    )
    emergency_stop_pub = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "-r",
            "2",
            "/commands/emergency_stop",
            "std_msgs/msg/Bool",
            "{data: false}",
        ],
        output="log",
    )

    wallbalancing = Node(
        package="wallfollowing2",
        executable="wallbalancing_node",
        output="screen",
        remappings=[
            ("/scan", scan_topic),
        ],
        parameters=[
            {"min_throttle": 0.28},
            {"max_throttle": 1.0},
            {"max_acceleration": 1.2},
            {"radius_lower": 0.8},
            {"radius_upper": 8.0},
            {"high_speed_steering_limit": 0.80},
            {"steering_slow_down": 2.2},
            {"barrier_size_relative": 0.04},
            {"barrier_lower_limit": 0.28},
            {"barrier_upper_limit": 1.4},
            {"barrier_exponent": 0.7},
            {"barrier_percentile": 65.0},
            {"barrier_speed_weight": 0.08},
            {"front_hard_stop_distance": 0.16},
            {"min_relative_speed": 0.30},
            {"controller_p": 4.4},
            {"controller_i": 0.03},
            {"controller_d": 0.08},
            {"corner_cutting": 1.25},
            {"straight_smoothing": 1.2},
            {"wall_balance_gain": 0.08},
            {"steering_sign": -1.0},
            {"usable_laser_range": 220.0},
        ],
    )

    return LaunchDescription(
        [
            start_lidar_driver_arg,
            start_uart_bridge_arg,
            lidar_ip_arg,
            lidar_port_arg,
            scan_topic_arg,
            laser_frame_id_arg,
            uart_command_mode_arg,
            uart_cmd_vel_topic_arg,
            uart_device_arg,
            uart_baud_rate_arg,
            uart_send_rate_arg,
            uart_command_timeout_arg,
            car_max_linear_speed_arg,
            car_max_steering_angle_arg,
            urg_driver,
            drive_parameters_multiplexer,
            car_controller,
            uart_actuator_bridge,
            drive_mode_pub,
            emergency_stop_pub,
            wallbalancing,
        ]
    )
