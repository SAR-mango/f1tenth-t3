import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
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


def generate_launch_description():
    world = LaunchConfiguration("world")
    use_wallfollowing = LaunchConfiguration("wallfollowing")
    use_control = LaunchConfiguration("control")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=_default_world_path(),
        description="Path to the Gazebo Fortress world file.",
    )
    wallfollowing_arg = DeclareLaunchArgument(
        "wallfollowing",
        default_value="true",
        description="Start wallfollowing2 ROS2 node.",
    )
    control_arg = DeclareLaunchArgument(
        "control",
        default_value="true",
        description="Start drive_parameters_multiplexer and car_controller.",
    )

    gz_sim = ExecuteProcess(
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

    drive_parameters_multiplexer = Node(
        package="car_control",
        executable="drive_parameters_multiplexer",
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(use_control),
    )

    car_controller = Node(
        package="car_control",
        executable="car_controller",
        parameters=[
            {"use_sim_time": True},
            {"publish_cmd_vel": True},
            {"max_linear_speed": 1.0},
            {"max_steering_angle": 0.5},
        ],
        output="screen",
        condition=IfCondition(use_control),
    )

    wallfollowing = Node(
        package="wallfollowing2",
        executable="wallfollowing_node",
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(use_wallfollowing),
    )

    return LaunchDescription(
        [
            world_arg,
            wallfollowing_arg,
            control_arg,
            gz_sim,
            bridge,
            drive_parameters_multiplexer,
            car_controller,
            wallfollowing,
        ]
    )
