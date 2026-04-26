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
    steering_plot_title = LaunchConfiguration("steering_plot_title")
    steering_axis_label = LaunchConfiguration("steering_axis_label")
    steering_zero_is_straight = LaunchConfiguration("steering_zero_is_straight")
    render_period_sec = LaunchConfiguration("render_period_sec")
    overlay_mode = LaunchConfiguration("overlay_mode")
    weighted_pairs_params_file = LaunchConfiguration("weighted_pairs_params_file")

    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="false",
        description="Launch RViz2 alongside the simulator-style dashboard when true.",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=_default_rviz_config_path(),
        description="RViz2 config used when start_rviz is true.",
    )
    scan_topic_arg = DeclareLaunchArgument(
        "scan_topic",
        default_value="/scan",
        description="LiDAR scan topic rendered in the dashboard forward/lateral view.",
    )
    motion_topic_arg = DeclareLaunchArgument(
        "motion_topic",
        default_value="/cmd_vel",
        description="Twist topic used for speed and steering history when stamped_motion_topic is idle.",
    )
    stamped_motion_topic_arg = DeclareLaunchArgument(
        "stamped_motion_topic",
        default_value="/telemetry/uart_command",
        description="TwistStamped topic preferred for command-history plotting when available.",
    )
    steering_plot_title_arg = DeclareLaunchArgument(
        "steering_plot_title",
        default_value="Steering Command",
        description="Title used for the steering history plot.",
    )
    steering_axis_label_arg = DeclareLaunchArgument(
        "steering_axis_label",
        default_value="Command",
        description="Y-axis label used for the steering history plot.",
    )
    steering_zero_is_straight_arg = DeclareLaunchArgument(
        "steering_zero_is_straight",
        default_value="false",
        description="Treat zero steering values as straight and hide them in the history plot.",
    )
    render_period_sec_arg = DeclareLaunchArgument(
        "render_period_sec",
        default_value="0.1",
        description="Dashboard redraw period in seconds.",
    )
    overlay_mode_arg = DeclareLaunchArgument(
        "overlay_mode",
        default_value="none",
        description="Optional LiDAR overlay mode. Use weighted_pairs_mmse for the MMSE cone overlay.",
    )
    weighted_pairs_params_file_arg = DeclareLaunchArgument(
        "weighted_pairs_params_file",
        default_value="",
        description="Optional weighted_pairs_mmse parameter YAML used to match dashboard overlay tuning.",
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
                "steering_plot_title": steering_plot_title,
                "steering_axis_label": steering_axis_label,
                "steering_zero_is_straight": steering_zero_is_straight,
                "render_period_sec": render_period_sec,
                "overlay_mode": overlay_mode,
                "weighted_pairs_params_file": weighted_pairs_params_file,
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
            steering_plot_title_arg,
            steering_axis_label_arg,
            steering_zero_is_straight_arg,
            render_period_sec_arg,
            overlay_mode_arg,
            weighted_pairs_params_file_arg,
            rviz,
            dashboard,
        ]
    )
