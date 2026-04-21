import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_world_path():
    repo_root = os.environ.get("F1TENTH_T3_ROOT", os.path.expanduser("~/f1tenth-t3"))
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
            "max_linear_speed:=1.35",
            "-p",
            "max_steering_angle:=0.62",
        ],
        output="screen",
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

    follow_the_gap = Node(
        package="wallfollowing2",
        executable="follow_the_gap_node",
        parameters=[
            {"use_sim_time": True},
            {"field_of_view_degrees": 220.0},
            {"max_range": 12.0},
            {"bubble_angle_degrees": 23.0},
            {"bubble_turn_gain_degrees": 10.0},
            {"bubble_max_angle_degrees": 34.0},
            {"side_clearance_min": 0.44},
            {"wall_bias_gain": 0.08},
            {"wall_bias_deadband_m": 0.14},
            {"steering_smoothing_alpha": 0.62},
            {"max_steering_step": 0.11},
            {"turn_intent_weight": 0.90},
            {"sharp_turn_steering_threshold": 0.36},
            {"sharp_turn_extra_steering_step": 0.06},
            {"sharp_turn_alpha_reduction": 0.18},
            {"corner_escape_turn_threshold_degrees": 30.0},
            {"corner_escape_front_distance": 1.20},
            {"corner_escape_wall_margin": 0.30},
            {"corner_escape_bias": 0.24},
            {"corner_escape_extra_steering_step": 0.08},
            {"corner_escape_alpha_reduction": 0.14},
            {"side_speed_floor": 0.40},
            {"gap_fallback_speed_scale": 0.45},
            {"hard_stop_distance": 0.28},
            {"front_stop_distance": 0.56},
            {"front_caution_distance": 2.30},
            {"front_window_degrees": 20.0},
            {"front_path_window_degrees": 10.0},
            {"front_path_steering_threshold_degrees": 18.0},
            {"front_stop_percentile": 12.0},
            {"front_stop_hold_frames": 3},
            {"steering_target_angle_degrees": 62.0},
            {"steering_slowdown_exponent": 1.15},
            {"min_speed": 0.20},
            {"max_speed": 1.08},
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
            follow_the_gap,
        ]
    )
