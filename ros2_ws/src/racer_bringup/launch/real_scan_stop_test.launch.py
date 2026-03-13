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

    # Scan-stop test behavior parameters.
    startup_delay_sec = LaunchConfiguration("startup_delay_sec")
    forward_speed_mps = LaunchConfiguration("forward_speed_mps")
    stop_distance_m = LaunchConfiguration("stop_distance_m")
    stop_hold_sec = LaunchConfiguration("stop_hold_sec")
    reverse_speed_mps = LaunchConfiguration("reverse_speed_mps")
    reverse_distance_m = LaunchConfiguration("reverse_distance_m")

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
        description="LaserScan topic used by test and adapter nodes.",
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

    startup_delay_arg = DeclareLaunchArgument("startup_delay_sec", default_value="6.0")
    forward_speed_arg = DeclareLaunchArgument("forward_speed_mps", default_value="0.6")
    stop_distance_arg = DeclareLaunchArgument("stop_distance_m", default_value="0.5")
    stop_hold_arg = DeclareLaunchArgument("stop_hold_sec", default_value="3.0")
    reverse_speed_arg = DeclareLaunchArgument("reverse_speed_mps", default_value="0.3")
    reverse_distance_arg = DeclareLaunchArgument("reverse_distance_m", default_value="0.5")

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

    test_behavior = Node(
        package="wallfollowing2",
        executable="scan_stop_reverse_test_node",
        output="screen",
        parameters=[
            {
                "scan_topic": scan_topic,
                "cmd_vel_topic": "/cmd_vel",
                "startup_delay_sec": startup_delay_sec,
                "forward_speed_mps": forward_speed_mps,
                "stop_distance_m": stop_distance_m,
                "stop_hold_sec": stop_hold_sec,
                "reverse_speed_mps": reverse_speed_mps,
                "reverse_distance_m": reverse_distance_m,
            }
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
            startup_delay_arg,
            forward_speed_arg,
            stop_distance_arg,
            stop_hold_arg,
            reverse_speed_arg,
            reverse_distance_arg,
            urg_driver,
            uart_actuator_bridge,
            test_behavior,
        ]
    )
