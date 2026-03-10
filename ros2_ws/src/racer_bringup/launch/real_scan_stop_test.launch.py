import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
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
    lidar_ip = LaunchConfiguration("lidar_ip")
    lidar_port = LaunchConfiguration("lidar_port")
    scan_topic = LaunchConfiguration("scan_topic")
    laser_frame_id = LaunchConfiguration("laser_frame_id")

    # Scan-stop test behavior parameters.
    startup_delay_sec = LaunchConfiguration("startup_delay_sec")
    forward_speed_mps = LaunchConfiguration("forward_speed_mps")
    stop_distance_m = LaunchConfiguration("stop_distance_m")
    stop_hold_sec = LaunchConfiguration("stop_hold_sec")
    reverse_speed_mps = LaunchConfiguration("reverse_speed_mps")
    reverse_distance_m = LaunchConfiguration("reverse_distance_m")

    start_lidar_driver_arg = DeclareLaunchArgument(
        "start_lidar_driver",
        default_value="true",
        description="If true, launch urg_node_driver for Hokuyo over Ethernet.",
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
        description="LaserScan topic used by test and adapter nodes.",
    )
    laser_frame_id_arg = DeclareLaunchArgument(
        "laser_frame_id",
        default_value="car/chassis/ust10lx",
        description="Frame id for published LaserScan messages.",
    )

    startup_delay_arg = DeclareLaunchArgument("startup_delay_sec", default_value="6.0")
    forward_speed_arg = DeclareLaunchArgument("forward_speed_mps", default_value="0.6")
    stop_distance_arg = DeclareLaunchArgument("stop_distance_m", default_value="0.5")
    stop_hold_arg = DeclareLaunchArgument("stop_hold_sec", default_value="3.0")
    reverse_speed_arg = DeclareLaunchArgument("reverse_speed_mps", default_value="0.3")
    reverse_distance_arg = DeclareLaunchArgument("reverse_distance_m", default_value="0.5")

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
    )

    drive_parameters_multiplexer = ExecuteProcess(
        cmd=[_car_control_exec("drive_parameters_multiplexer")],
        output="screen",
    )

    car_controller = ExecuteProcess(
        cmd=[
            _car_control_exec("car_controller"),
            "--ros-args",
            "-p",
            "publish_cmd_vel:=false",
        ],
        output="screen",
    )

    # Arm autonomous mode and continuously clear emergency stop.
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

    test_behavior = Node(
        package="wallfollowing2",
        executable="scan_stop_reverse_test_node",
        output="screen",
        parameters=[
            {
                "scan_topic": scan_topic,
                "cmd_vel_topic": "/cmd_vel",
                "startup_delay_sec": startup_delay_sec,
                "forward_speed_mps": forward_speed_mps,
                "stop_distance_m": stop_distance_m,
                "stop_hold_sec": stop_hold_sec,
                "reverse_speed_mps": reverse_speed_mps,
                "reverse_distance_m": reverse_distance_m,
            }
        ],
    )

    cmd_vel_to_drive_param = Node(
        package="wallfollowing2",
        executable="cmd_vel_to_drive_param_real_node",
        output="screen",
        parameters=[
            {
                "cmd_vel_topic": "/cmd_vel",
                "drive_param_topic": "/input/drive_param/autonomous",
                "angular_input_mode": "turning_radius_m",
                # Ackermann defaults from the active world file plugin.
                "wheel_base_m": 0.300568,
                "kingpin_width_m": 0.2,
                "steering_limit_rad": 0.7,
            }
        ],
    )

    return LaunchDescription(
        [
            start_lidar_driver_arg,
            lidar_ip_arg,
            lidar_port_arg,
            scan_topic_arg,
            laser_frame_id_arg,
            startup_delay_arg,
            forward_speed_arg,
            stop_distance_arg,
            stop_hold_arg,
            reverse_speed_arg,
            reverse_distance_arg,
            urg_driver,
            drive_parameters_multiplexer,
            car_controller,
            drive_mode_pub,
            emergency_stop_pub,
            cmd_vel_to_drive_param,
            test_behavior,
        ]
    )
