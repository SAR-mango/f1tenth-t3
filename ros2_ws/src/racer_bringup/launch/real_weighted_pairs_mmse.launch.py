import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    start_dashboard = LaunchConfiguration("start_dashboard")
    dashboard_start_rviz = LaunchConfiguration("dashboard_start_rviz")
    lidar_ip = LaunchConfiguration("lidar_ip")
    lidar_port = LaunchConfiguration("lidar_port")
    scan_topic = LaunchConfiguration("scan_topic")
    laser_frame_id = LaunchConfiguration("laser_frame_id")

    uart_cmd_vel_topic = LaunchConfiguration("uart_cmd_vel_topic")
    uart_device = LaunchConfiguration("uart_device")
    uart_baud_rate = LaunchConfiguration("uart_baud_rate")
    uart_send_rate_hz = LaunchConfiguration("uart_send_rate_hz")
    uart_command_timeout_sec = LaunchConfiguration("uart_command_timeout_sec")

    steering_radius_sign = LaunchConfiguration("steering_radius_sign")
    stop_on_algorithm_fallback = LaunchConfiguration("stop_on_algorithm_fallback")
    log_status_interval_sec = LaunchConfiguration("log_status_interval_sec")

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
    start_dashboard_arg = DeclareLaunchArgument(
        "start_dashboard",
        default_value="true",
        description="If true, launch the dashboard alongside the real weighted-pairs stack.",
    )
    dashboard_start_rviz_arg = DeclareLaunchArgument(
        "dashboard_start_rviz",
        default_value="false",
        description="Also launch RViz inside the included dashboard bringup.",
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
        description="LaserScan topic used by the weighted_pairs_mmse node.",
    )
    laser_frame_id_arg = DeclareLaunchArgument(
        "laser_frame_id",
        default_value="car/chassis/ust10lx",
        description="Frame id for published LaserScan messages.",
    )

    uart_cmd_vel_topic_arg = DeclareLaunchArgument(
        "uart_cmd_vel_topic",
        default_value="/cmd_vel",
        description="Twist topic forwarded over UART.",
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

    steering_radius_sign_arg = DeclareLaunchArgument(
        "steering_radius_sign",
        default_value="1.0",
        description=(
            "Multiplier applied before publishing the algorithm radius command. "
            "Keep 1.0 for the current positive-left / negative-right convention."
        ),
    )
    stop_on_algorithm_fallback_arg = DeclareLaunchArgument(
        "stop_on_algorithm_fallback",
        default_value="true",
        description="Force a zero-speed command whenever the algorithm reports a fallback state.",
    )
    log_status_interval_arg = DeclareLaunchArgument(
        "log_status_interval_sec",
        default_value="1.0",
        description="How often the weighted_pairs_mmse node logs its status string.",
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

    uart_actuator_bridge = ExecuteProcess(
        cmd=[
            _car_control_exec("uart_actuator_bridge"),
            "--ros-args",
            "-p",
            "command_mode:=cmd_vel",
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
            "-p",
            "respect_drive_mode_and_estop:=true",
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

    weighted_pairs_mmse = Node(
        package="wallfollowing2",
        executable="weighted_pairs_mmse_node",
        output="screen",
        parameters=[
            {
                "scan_topic": scan_topic,
                "cmd_vel_topic": uart_cmd_vel_topic,
                "steering_radius_sign": steering_radius_sign,
                "stop_on_algorithm_fallback": stop_on_algorithm_fallback,
                "log_status_interval_sec": log_status_interval_sec,
            }
        ],
    )

    dashboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(os.path.dirname(__file__), "dashboard.launch.py")
        ),
        launch_arguments={
            "start_rviz": dashboard_start_rviz,
            "scan_topic": scan_topic,
            "motion_topic": uart_cmd_vel_topic,
            "stamped_motion_topic": "/telemetry/uart_command",
        }.items(),
        condition=IfCondition(start_dashboard),
    )

    return LaunchDescription(
        [
            start_lidar_driver_arg,
            start_uart_bridge_arg,
            start_dashboard_arg,
            dashboard_start_rviz_arg,
            lidar_ip_arg,
            lidar_port_arg,
            scan_topic_arg,
            laser_frame_id_arg,
            uart_cmd_vel_topic_arg,
            uart_device_arg,
            uart_baud_rate_arg,
            uart_send_rate_arg,
            uart_command_timeout_arg,
            steering_radius_sign_arg,
            stop_on_algorithm_fallback_arg,
            log_status_interval_arg,
            urg_driver,
            uart_actuator_bridge,
            drive_mode_pub,
            emergency_stop_pub,
            weighted_pairs_mmse,
            dashboard,
        ]
    )
