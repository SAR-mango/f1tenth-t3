import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_world_path():
    repo_root = os.environ.get("F1TENTH_T3_ROOT", "/home/erk/f1tenth-t3")
    return os.path.join(
        repo_root,
        "ros_ws",
        "src",
        "simulation",
        "racer_world",
        "worlds",
        "racetrack_decorated_2_hokuyo.world",
    )


def _default_rviz_config_path():
    repo_root = os.environ.get("F1TENTH_T3_ROOT", "/home/erk/f1tenth-t3")
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
        get_package_share_directory("racer_bringup"), "config", "rviz2_lidar.rviz"
    )


def generate_launch_description():
    world = LaunchConfiguration("world")
    base_frame = LaunchConfiguration("base_frame")
    scan_frame = LaunchConfiguration("scan_frame")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=_default_world_path(),
        description="Path to the Gazebo Fortress world file.",
    )
    base_frame_arg = DeclareLaunchArgument(
        "base_frame",
        default_value="base_link",
        description="Parent frame for static TF.",
    )
    scan_frame_arg = DeclareLaunchArgument(
        "scan_frame",
        default_value="car/chassis/ust10lx",
        description="Laser frame used by /scan frame_id.",
    )

    ign_sim = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", world],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/model/car/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        remappings=[
            ("/model/car/cmd_vel", "/cmd_vel"),
        ],
        output="screen",
    )

    rviz_config = _default_rviz_config_path()
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    # RViz needs TF frames to exist even when visualizing scan in sensor frame.
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", base_frame,
            "--child-frame-id", scan_frame,
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            world_arg,
            base_frame_arg,
            scan_frame_arg,
            ign_sim,
            bridge,
            static_tf,
            rviz,
        ]
    )
