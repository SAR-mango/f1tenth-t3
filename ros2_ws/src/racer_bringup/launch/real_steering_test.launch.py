import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
    start_uart_bridge = LaunchConfiguration("start_uart_bridge")
    uart_command_mode = LaunchConfiguration("uart_command_mode")
    uart_cmd_vel_topic = LaunchConfiguration("uart_cmd_vel_topic")
    uart_device = LaunchConfiguration("uart_device")
    uart_baud_rate = LaunchConfiguration("uart_baud_rate")
    uart_send_rate_hz = LaunchConfiguration("uart_send_rate_hz")
    uart_command_timeout_sec = LaunchConfiguration("uart_command_timeout_sec")

    startup_delay_sec = LaunchConfiguration("startup_delay_sec")
    linear_speed_mps = LaunchConfiguration("linear_speed_mps")
    step_duration_sec = LaunchConfiguration("step_duration_sec")
    inter_step_wait_sec = LaunchConfiguration("inter_step_wait_sec")
    turn_right = LaunchConfiguration("turn_right")
    control_rate_hz = LaunchConfiguration("control_rate_hz")

    start_uart_bridge_arg = DeclareLaunchArgument(
        "start_uart_bridge",
        default_value="true",
        description="If true, launch the UART actuator bridge for Jetson serial output.",
    )
    uart_command_mode_arg = DeclareLaunchArgument(
        "uart_command_mode",
        default_value="cmd_vel",
        description="UART bridge input mode: cmd_vel or actuator_topics.",
    )
    uart_cmd_vel_topic_arg = DeclareLaunchArgument(
        "uart_cmd_vel_topic",
        default_value="/cmd_vel",
        description="Twist topic forwarded over UART when using cmd_vel mode.",
    )
    uart_device_arg = DeclareLaunchArgument(
        "uart_device",
        default_value="/dev/ttyTHS1",
        description="Jetson UART device used for actuator output.",
    )
    uart_baud_rate_arg = DeclareLaunchArgument(
        "uart_baud_rate",
        default_value="115200",
        description="UART baud rate for actuator output.",
    )
    uart_send_rate_arg = DeclareLaunchArgument(
        "uart_send_rate_hz",
        default_value="40.0",
        description="How often the UART bridge transmits the latest command frame.",
    )
    uart_command_timeout_arg = DeclareLaunchArgument(
        "uart_command_timeout_sec",
        default_value="0.5",
        description="If no new command arrives within this timeout, the UART bridge sends zeros.",
    )

    startup_delay_arg = DeclareLaunchArgument(
        "startup_delay_sec",
        default_value="3.0",
        description="How long to wait before starting the steering-step sequence.",
    )
    linear_speed_arg = DeclareLaunchArgument(
        "linear_speed_mps",
        default_value="0.6",
        description="Constant forward speed used for all steering-radius steps.",
    )
    step_duration_arg = DeclareLaunchArgument(
        "step_duration_sec",
        default_value="40.0",
        description="Duration of each constant-radius steering plateau.",
    )
    inter_step_wait_arg = DeclareLaunchArgument(
        "inter_step_wait_sec",
        default_value="10.0",
        description="How long to remain stopped between successive steering-radius commands.",
    )
    turn_right_arg = DeclareLaunchArgument(
        "turn_right",
        default_value="false",
        description="If true, run the steering schedule turning right instead of left.",
    )
    control_rate_arg = DeclareLaunchArgument(
        "control_rate_hz",
        default_value="20.0",
        description="How often the node republishes the current test command.",
    )

    uart_actuator_bridge = ExecuteProcess(
        cmd=[
            _car_control_exec("uart_actuator_bridge"),
            "--ros-args",
            "-p",
            ["command_mode:=", uart_command_mode],
            "-p",
            ["cmd_vel_topic:=", uart_cmd_vel_topic],
            "-p",
            ["uart_device:=", uart_device],
            "-p",
            ["baud_rate:=", uart_baud_rate],
            "-p",
            ["send_rate_hz:=", uart_send_rate_hz],
            "-p",
            ["command_timeout_sec:=", uart_command_timeout_sec],
        ],
        output="screen",
        condition=IfCondition(start_uart_bridge),
    )

    test_behavior = Node(
        package="wallfollowing2",
        executable="steering_test_node",
        output="screen",
        parameters=[
            {
                "cmd_vel_topic": uart_cmd_vel_topic,
                "startup_delay_sec": startup_delay_sec,
                "linear_speed_mps": linear_speed_mps,
                "step_duration_sec": step_duration_sec,
                "inter_step_wait_sec": inter_step_wait_sec,
                "turn_right": ParameterValue(turn_right, value_type=bool),
                "control_rate_hz": control_rate_hz,
            }
        ],
    )

    return LaunchDescription(
        [
            start_uart_bridge_arg,
            uart_command_mode_arg,
            uart_cmd_vel_topic_arg,
            uart_device_arg,
            uart_baud_rate_arg,
            uart_send_rate_arg,
            uart_command_timeout_arg,
            startup_delay_arg,
            linear_speed_arg,
            step_duration_arg,
            inter_step_wait_arg,
            turn_right_arg,
            control_rate_arg,
            uart_actuator_bridge,
            test_behavior,
        ]
    )
