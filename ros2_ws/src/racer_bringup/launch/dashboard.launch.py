import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    scan_topic = LaunchConfiguration("scan_topic")
    motion_topic = LaunchConfiguration("motion_topic")
    stamped_motion_topic = LaunchConfiguration("stamped_motion_topic")
    render_period_sec = LaunchConfiguration("render_period_sec")

    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="false",
        description="Launch RViz2 alongside the simplified dashboard when true.",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=_default_rviz_config_path(),
        description="RViz2 config used when start_rviz is true.",
    )
    scan_topic_arg = DeclareLaunchArgument(
        "scan_topic",
        default_value="/scan",
        description="LiDAR scan topic rendered in the dashboard XY view.",
    )
    motion_topic_arg = DeclareLaunchArgument(
        "motion_topic",
        default_value="/cmd_vel",
        description="Twist topic used for the velocity and steering numeric readouts.",
    )
    stamped_motion_topic_arg = DeclareLaunchArgument(
        "stamped_motion_topic",
        default_value="/telemetry/uart_command",
        description="TwistStamped topic preferred for numeric readouts when available.",
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
        condition=IfCondition(start_rviz),
        output="screen",
    )

    dashboard = Node(
        package="racer_bringup",
        executable="dashboard_node",
        output="screen",
        parameters=[
            {
                "scan_topic": scan_topic,
                "motion_topic": motion_topic,
                "stamped_motion_topic": stamped_motion_topic,
                "render_period_sec": render_period_sec,
            }
        ],
    )

    return LaunchDescription(
        [
            start_rviz_arg,
            rviz_config_arg,
            scan_topic_arg,
            motion_topic_arg,
            stamped_motion_topic_arg,
            render_period_sec_arg,
            rviz,
            dashboard,
        ]
    )
