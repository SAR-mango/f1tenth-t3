import os

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


def _car_control_exec(name: str) -> str:
    repo_root = os.environ.get("F1TENTH_T3_ROOT", "/home/erk/f1tenth-t3")
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
            "/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            "/model/car/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        remappings=[
            ("/model/car/cmd_vel", "/cmd_vel"),
        ],
        output="screen",
    )

    drive_parameters_multiplexer = ExecuteProcess(
        cmd=[
            _car_control_exec("drive_parameters_multiplexer"),
            "--ros-args",
            "-p",
            "use_sim_time:=true",
        ],
        output="screen",
    )

    car_controller = ExecuteProcess(
        cmd=[
            _car_control_exec("car_controller"),
            "--ros-args",
            "-p",
            "use_sim_time:=true",
            "-p",
            "publish_cmd_vel:=true",
            "-p",
            "max_linear_speed:=2.9",
            "-p",
            "max_steering_angle:=0.62",
        ],
        output="screen",
    )

    # Arm autonomous drive mode and clear emergency stop continuously.
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

    wallfollowing = Node(
        package="wallfollowing2",
        executable="wallfollowing_node",
        parameters=[
            {"use_sim_time": True},
            {"left_distance_setpoint": 0.25},
            {"lookahead_distance": 0.70},
            {"kp": 2.0},
            {"kd": 0.9},
            {"steering_limit": 1.0},
            {"steering_sign": -1.0},
            {"min_throttle": 0.58},
            {"max_throttle": 1.0},
            {"steering_speed_reduction": 0.32},
            {"max_acceleration": 2.8},
            {"front_stop_distance": 0.18},
            {"front_slow_distance": 0.55},
            {"front_angle_half_width_deg": 10.0},
            {"usable_laser_range": 220.0},
            {"wall_sample_min_x": 0.05},
            {"wall_sample_max_x": 1.10},
            {"wall_sample_min_y": 0.10},
            {"wall_sample_max_y": 1.40},
            {"sample_stride": 2},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            world_arg,
            ign_sim,
            bridge,
            drive_parameters_multiplexer,
            car_controller,
            drive_mode_pub,
            emergency_stop_pub,
            wallfollowing,
        ]
    )
