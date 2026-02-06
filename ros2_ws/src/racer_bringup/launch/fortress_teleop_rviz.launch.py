import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_world_path():
    here = os.path.dirname(__file__)
    repo_root = os.path.abspath(os.path.join(here, "..", "..", "..", ".."))
    return os.path.join(
        repo_root,
        "ros_ws",
        "src",
        "simulation",
        "racer_world",
        "worlds",
        "racetrack_decorated_2_hokuyo.world",
    )


def generate_launch_description():
    world = LaunchConfiguration("world")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=_default_world_path(),
        description="Path to the Gazebo Fortress world file.",
    )

    ign_sim = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", world],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan",
            "/model/car/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock",
        ],
        remappings=[
            ("/model/car/cmd_vel", "/cmd_vel"),
        ],
        output="screen",
    )

    rviz_config = os.path.join(
        get_package_share_directory("racer_bringup"), "config", "rviz2_lidar.rviz"
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription(
        [
            world_arg,
            ign_sim,
            bridge,
            rviz,
        ]
    )
