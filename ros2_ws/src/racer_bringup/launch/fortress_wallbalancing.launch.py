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
            "max_linear_speed:=2.4",
            "-p",
            "max_steering_angle:=0.54",
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

    wallbalancing = Node(
        package="wallfollowing2",
        executable="wallbalancing_node",
        parameters=[
            {"use_sim_time": True},
            {"min_throttle": 0.28},
            {"max_throttle": 1.0},
            {"max_acceleration": 1.2},
            {"radius_lower": 0.8},
            {"radius_upper": 8.0},
            {"high_speed_steering_limit": 0.80},
            {"steering_slow_down": 2.2},
            {"barrier_size_relative": 0.04},
            {"barrier_lower_limit": 0.28},
            {"barrier_upper_limit": 1.4},
            {"barrier_exponent": 0.7},
            {"barrier_percentile": 65.0},
            {"barrier_speed_weight": 0.08},
            {"front_hard_stop_distance": 0.16},
            {"min_relative_speed": 0.30},
            {"controller_p": 4.4},
            {"controller_i": 0.03},
            {"controller_d": 0.08},
            {"corner_cutting": 1.25},
            {"straight_smoothing": 1.2},
            {"wall_balance_gain": 0.08},
            {"steering_sign": -1.0},
            {"usable_laser_range": 220.0},
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
            wallbalancing,
        ]
    )
