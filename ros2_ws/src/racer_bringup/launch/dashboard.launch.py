import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_rviz_config_path():
    repo_root = os.environ.get("F1TENTH_T3_ROOT", os.path.expanduser("~/f1tenth-t3"))
    source_cfg = os.path.join(
        repo_root,
        "ros2_ws",
        "src",
        "racer_bringup",
        "config",
        "rviz2_lidar.rviz",
    )
    if os.path.exists(source_cfg):
        return source_cfg
    return os.path.join(
        get_package_share_directory("racer_bringup"),
        "config",
        "rviz2_lidar.rviz",
    )


def generate_launch_description():
    rviz_config = LaunchConfiguration("rviz_config")
    scan_topic = LaunchConfiguration("scan_topic")
    uart_telemetry_topic = LaunchConfiguration("uart_telemetry_topic")
    uart_frame_topic = LaunchConfiguration("uart_frame_topic")
    uart_stale_topic = LaunchConfiguration("uart_stale_topic")
    measured_speed_topic = LaunchConfiguration("measured_speed_topic")
    measured_steering_topic = LaunchConfiguration("measured_steering_topic")
    drive_mode_topic = LaunchConfiguration("drive_mode_topic")
    emergency_stop_topic = LaunchConfiguration("emergency_stop_topic")
    window_sec = LaunchConfiguration("window_sec")
    render_period_sec = LaunchConfiguration("render_period_sec")

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=_default_rviz_config_path(),
        description="RViz2 config used for the live lidar view.",
    )
    scan_topic_arg = DeclareLaunchArgument(
        "scan_topic",
        default_value="/scan",
        description="Real lidar scan topic shown in RViz2 and scan status.",
    )
    uart_telemetry_topic_arg = DeclareLaunchArgument(
        "uart_telemetry_topic",
        default_value="/telemetry/uart_command",
        description="TwistStamped topic published by uart_actuator_bridge for transmitted commands.",
    )
    uart_frame_topic_arg = DeclareLaunchArgument(
        "uart_frame_topic",
        default_value="/telemetry/uart_frame",
        description="String topic carrying the latest CSV frame written to UART.",
    )
    uart_stale_topic_arg = DeclareLaunchArgument(
        "uart_stale_topic",
        default_value="/telemetry/uart_command_stale",
        description="Bool topic that marks timeout-forced zero UART frames.",
    )
    measured_speed_topic_arg = DeclareLaunchArgument(
        "measured_speed_topic",
        default_value="/telemetry/measured_speed",
        description="Future Float64 speed feedback topic for the dashboard template.",
    )
    measured_steering_topic_arg = DeclareLaunchArgument(
        "measured_steering_topic",
        default_value="/telemetry/measured_steering",
        description="Future Float64 steering feedback topic for the dashboard template.",
    )
    drive_mode_topic_arg = DeclareLaunchArgument(
        "drive_mode_topic",
        default_value="/commands/drive_mode",
        description="Drive mode topic for status display.",
    )
    emergency_stop_topic_arg = DeclareLaunchArgument(
        "emergency_stop_topic",
        default_value="/commands/emergency_stop",
        description="Emergency stop topic for status display.",
    )
    window_sec_arg = DeclareLaunchArgument(
        "window_sec",
        default_value="60.0",
        description="Time window kept in the live plots before old samples are pruned.",
    )
    render_period_sec_arg = DeclareLaunchArgument(
        "render_period_sec",
        default_value="0.1",
        description="Dashboard redraw period in seconds.",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    dashboard = Node(
        package="racer_bringup",
        executable="dashboard_node",
        output="screen",
        parameters=[
            {
                "scan_topic": scan_topic,
                "uart_telemetry_topic": uart_telemetry_topic,
                "uart_frame_topic": uart_frame_topic,
                "uart_stale_topic": uart_stale_topic,
                "measured_speed_topic": measured_speed_topic,
                "measured_steering_topic": measured_steering_topic,
                "drive_mode_topic": drive_mode_topic,
                "emergency_stop_topic": emergency_stop_topic,
                "window_sec": window_sec,
                "render_period_sec": render_period_sec,
            }
        ],
    )

    return LaunchDescription(
        [
            rviz_config_arg,
            scan_topic_arg,
            uart_telemetry_topic_arg,
            uart_frame_topic_arg,
            uart_stale_topic_arg,
            measured_speed_topic_arg,
            measured_steering_topic_arg,
            drive_mode_topic_arg,
            emergency_stop_topic_arg,
            window_sec_arg,
            render_period_sec_arg,
            rviz,
            dashboard,
        ]
    )
