import math

import rclpy
from drive_msgs.msg import DriveParam
from geometry_msgs.msg import Twist
from rclpy.node import Node


def clamp(value, lower, upper):
    return min(upper, max(lower, value))


class CmdVelToDriveParamRealNode(Node):
    """
    Convert high-level cmd_vel commands to normalized DriveParam for car_control.

    Supported input modes for cmd_vel.angular.z:
    - turning_radius_m: signed turning radius in meters (0 means straight)
    - steering_angle_rad: signed steering angle in radians

    The turning-radius conversion uses the Ackermann bicycle-equivalent model
    with defaults matching the current world plugin configuration.
    """

    def __init__(self):
        super().__init__("cmd_vel_to_drive_param_real")

        # Topic plumbing
        self.cmd_vel_topic = str(self.declare_parameter("cmd_vel_topic", "/cmd_vel").value)
        self.drive_param_topic = str(
            self.declare_parameter("drive_param_topic", "/input/drive_param/autonomous").value
        )
        self.angular_input_mode = str(
            self.declare_parameter("angular_input_mode", "turning_radius_m").value
        )

        # Ackermann geometry defaults from racetrack_decorated_2_hokuyo.world
        # (AckermannSteering plugin block).
        self.wheel_base_m = float(self.declare_parameter("wheel_base_m", 0.300568).value)
        self.kingpin_width_m = float(self.declare_parameter("kingpin_width_m", 0.2).value)
        self.steering_limit_rad = float(self.declare_parameter("steering_limit_rad", 0.7).value)
        self.turning_radius_zero_epsilon_m = float(
            self.declare_parameter("turning_radius_zero_epsilon_m", 1e-3).value
        )

        # Vehicle + actuator mapping (legacy stack-compatible).
        self.max_rpm_electrical = float(self.declare_parameter("max_rpm_electrical", 20000.0).value)
        self.erpm_to_speed = float(self.declare_parameter("erpm_to_speed", (0.098 * math.pi * 3.0) / 60.0).value)
        self.transmission = float(self.declare_parameter("transmission", 20.0).value)
        self.steering_to_servo_offset = float(
            self.declare_parameter("steering_to_servo_offset", 0.5).value
        )
        self.steering_to_servo_gain = float(
            self.declare_parameter("steering_to_servo_gain", -3.0 / math.pi).value
        )
        self.max_servo_position = float(self.declare_parameter("max_servo_position", 1.0).value)
        self.speed_sign = float(self.declare_parameter("speed_sign", 1.0).value)
        self.steering_sign = float(self.declare_parameter("steering_sign", 1.0).value)

        self.cmd_sub = self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        self.drive_pub = self.create_publisher(DriveParam, self.drive_param_topic, 10)

        self.get_logger().info(
            "cmd_vel->DriveParam adapter ready. mode=%s cmd_vel=%s drive_param=%s wheel_base=%.6f "
            "steering_limit=%.3f kingpin_width=%.3f"
            % (
                self.angular_input_mode,
                self.cmd_vel_topic,
                self.drive_param_topic,
                self.wheel_base_m,
                self.steering_limit_rad,
                self.kingpin_width_m,
            )
        )

    def _min_turning_radius(self):
        if self.steering_limit_rad <= 1e-6:
            return float("inf")
        tangent = math.tan(abs(self.steering_limit_rad))
        if abs(tangent) <= 1e-9:
            return float("inf")
        return abs(self.wheel_base_m / tangent)

    def _steering_angle_from_turning_radius(self, turning_radius_m):
        # By convention here, turning_radius_m == 0 means straight driving.
        if (not math.isfinite(turning_radius_m)) or (
            abs(turning_radius_m) <= self.turning_radius_zero_epsilon_m
        ):
            return 0.0

        min_radius = self._min_turning_radius()
        radius = turning_radius_m
        if math.isfinite(min_radius) and abs(radius) < min_radius:
            radius = math.copysign(min_radius, radius)

        steering = math.atan(self.wheel_base_m / abs(radius))
        if radius < 0.0:
            steering *= -1.0
        return clamp(steering, -abs(self.steering_limit_rad), abs(self.steering_limit_rad))

    def _relative_speed_from_mps(self, speed_mps):
        speed = self.speed_sign * speed_mps
        if abs(self.max_rpm_electrical) < 1e-6 or abs(self.erpm_to_speed) < 1e-9:
            self.get_logger().error("Invalid speed conversion parameters; publishing zero speed.")
            return 0.0
        erpm_speed = speed * self.transmission / self.erpm_to_speed
        return clamp(erpm_speed / self.max_rpm_electrical, -1.0, 1.0)

    def _relative_angle_from_steering_angle(self, steering_angle_rad):
        steering = self.steering_sign * steering_angle_rad
        steering = clamp(steering, -abs(self.steering_limit_rad), abs(self.steering_limit_rad))

        if abs(self.max_servo_position) < 1e-9:
            self.get_logger().error("Invalid max_servo_position; publishing zero steering.")
            return 0.0

        servo_data = steering * self.steering_to_servo_gain + self.steering_to_servo_offset
        servo_data = clamp(servo_data, 0.0, self.max_servo_position)
        relative_angle = (servo_data * 2.0 - self.max_servo_position) / self.max_servo_position
        return clamp(relative_angle, -1.0, 1.0)

    def cmd_vel_callback(self, message):
        if self.angular_input_mode == "steering_angle_rad":
            steering_angle = float(message.angular.z)
        else:
            steering_angle = self._steering_angle_from_turning_radius(float(message.angular.z))

        drive_msg = DriveParam()
        drive_msg.velocity = float(self._relative_speed_from_mps(float(message.linear.x)))
        drive_msg.angle = float(self._relative_angle_from_steering_angle(steering_angle))
        self.drive_pub.publish(drive_msg)


def main():
    rclpy.init()
    node = CmdVelToDriveParamRealNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
