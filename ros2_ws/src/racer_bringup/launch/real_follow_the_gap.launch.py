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
    start_dashboard_arg = DeclareLaunchArgument(
        "start_dashboard",
        default_value="true",
        description="If true, launch the dashboard alongside the real follow-the-gap stack.",
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
        description="LaserScan topic used by the follow-the-gap node.",
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
            "5",
            "-t",
            "3",
            "/commands/emergency_stop",
            "std_msgs/msg/Bool",
            "{data: false}",
            "--keep-alive",
            "1.0",
        ],
        output="log",
    )

    follow_the_gap = Node(
        package="wallfollowing2",
        executable="follow_the_gap_node",
        output="screen",
        parameters=[
            {"scan_topic": scan_topic},
            {"field_of_view_degrees": 220.0},
            {"max_range": 10.0},
            {"bubble_angle_degrees": 24.0},
            {"bubble_turn_gain_degrees": 12.0},
            {"side_clearance_min": 0.42},
            {"wall_bias_gain": 0.07},
            {"wall_bias_deadband_m": 0.14},
            {"steering_smoothing_alpha": 0.74},
            {"max_steering_step": 0.07},
            {"turn_intent_weight": 0.72},
            {"sharp_turn_steering_threshold": 0.36},
            {"sharp_turn_extra_steering_step": 0.06},
            {"sharp_turn_alpha_reduction": 0.18},
            {"corner_escape_turn_threshold_degrees": 30.0},
            {"corner_escape_front_distance": 1.20},
            {"corner_escape_wall_margin": 0.30},
            {"corner_escape_bias": 0.24},
            {"corner_escape_extra_steering_step": 0.08},
            {"corner_escape_alpha_reduction": 0.14},
            {"side_speed_floor": 0.45},
            {"hard_stop_distance": 0.24},
            {"front_stop_distance": 0.48},
            {"front_caution_distance": 1.60},
            {"front_window_degrees": 14.0},
            {"front_path_window_degrees": 10.0},
            {"front_path_steering_threshold_degrees": 18.0},
            {"front_stop_percentile": 5.0},
            {"front_stop_hold_frames": 4},
            {"steering_slowdown_exponent": 1.25},
            {"min_speed": 0.18},
            {"max_speed": 0.85},
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
            "steering_plot_title": "Steering Angle Command",
            "steering_axis_label": "Angle (rad)",
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
            follow_the_gap,
            dashboard,
        ]
    )
