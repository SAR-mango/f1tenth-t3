import math
from types import SimpleNamespace

import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from drive_msgs.msg import DriveParam

from .circle import Circle, Point
from .rviz_geometry import (
    RVIZ_TOPIC,
    show_circle_in_rviz,
    show_line_in_rviz,
)


PARAM_DEFAULTS = {
    "min_throttle": 0.2,
    "max_throttle": 1.0,
    "radius_lower": 2.0,
    "radius_upper": 30.0,
    "steering_slow_down": 4.0,
    "steering_slow_down_dead_zone": 0.2,
    "high_speed_steering_limit": 0.5,
    "high_speed_steering_limit_dead_zone": 0.2,
    "max_acceleration": 0.4,
    "corner_cutting": 1.4,
    "straight_smoothing": 1.0,
    "barrier_size_realtive": 0.1,
    "barrier_lower_limit": 1.0,
    "barrier_upper_limit": 15.0,
    "barrier_exponent": 1.4,
    "controller_p": 4.0,
    "controller_i": 0.2,
    "controller_d": 0.02,
    "usable_laser_range": 220.0,
}

TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous"
TOPIC_LASER_SCAN = "/scan"


class PIDController:
    def __init__(self, p, i, d, anti_windup=0.2):
        self.p = p
        self.i = i
        self.d = d
        self.anti_windup = anti_windup
        self.integral = 0.0
        self.previous_error = 0.0

    def update_and_get_correction(self, error, delta_time):
        self.integral += error * delta_time
        self.integral = min(self.anti_windup, max(-self.anti_windup, self.integral))
        derivative = (error - self.previous_error) / delta_time
        self.previous_error = error
        return self.p * error + self.i * self.integral + self.d * derivative


def map_value(in_lower, in_upper, out_lower, out_upper, value):
    result = out_lower + (out_upper - out_lower) * (
        (value - in_lower) / (in_upper - in_lower)
    )
    return min(out_upper, max(out_lower, result))


class WallFollowingNode(Node):
    def __init__(self):
        super().__init__("wallfollowing")
        self._declare_parameters()
        self.params = SimpleNamespace()
        self._load_parameters()
        self.add_on_set_parameters_callback(self._on_params)

        self.drive_pub = self.create_publisher(DriveParam, TOPIC_DRIVE_PARAMETERS, 1)
        self.scan_sub = self.create_subscription(
            LaserScan, TOPIC_LASER_SCAN, self.laser_callback, 10
        )
        self.marker_pub = self.create_publisher(Marker, RVIZ_TOPIC, 1)

        self.pid = PIDController(
            self.params.controller_p,
            self.params.controller_i,
            self.params.controller_d,
        )

        self.last_speed = 0.0
        self.last_scan_time = None

    def _declare_parameters(self):
        for name, default in PARAM_DEFAULTS.items():
            self.declare_parameter(name, default)

    def _load_parameters(self):
        for name in PARAM_DEFAULTS.keys():
            setattr(self.params, name, self.get_parameter(name).value)
        self.get_logger().debug("Loaded parameters.")

    def _on_params(self, _params):
        self._load_parameters()
        self.pid.p = self.params.controller_p
        self.pid.i = self.params.controller_i
        self.pid.d = self.params.controller_d
        return SetParametersResult(successful=True)

    def drive(self, angle, velocity):
        message = DriveParam()
        message.angle = float(angle)
        message.velocity = float(velocity)
        self.drive_pub.publish(message)

    def get_scan_as_cartesian(self, laser_scan):
        ranges = np.array(laser_scan.ranges)

        angles = np.linspace(
            laser_scan.angle_min,
            laser_scan.angle_max,
            ranges.shape[0],
        )

        laser_range = laser_scan.angle_max - laser_scan.angle_min
        usable_range = math.radians(self.params.usable_laser_range)
        if usable_range < laser_range:
            skip_left = int(
                (-laser_scan.angle_min - usable_range / 2)
                / laser_range
                * ranges.shape[0]
            )
            skip_right = int(
                (laser_scan.angle_max - usable_range / 2)
                / laser_range
                * ranges.shape[0]
            )
            angles = angles[skip_left:-1 - skip_right]
            ranges = ranges[skip_left:-1 - skip_right]

        inf_mask = np.isinf(ranges)
        if inf_mask.any():
            ranges = ranges[~inf_mask]
            angles = angles[~inf_mask]

        points = np.zeros((ranges.shape[0], 2))
        points[:, 0] = -np.sin(angles) * ranges
        points[:, 1] = np.cos(angles) * ranges
        return points

    def find_left_right_border(self, points, margin_relative=0.1):
        margin = int(points.shape[0] * margin_relative)
        relative = points[margin + 1:-margin, :] - points[margin:-margin - 1, :]
        distances = np.linalg.norm(relative, axis=1)
        return margin + np.argmax(distances) + 1

    def follow_walls(self, left_circle, right_circle, barrier, delta_time):
        prediction_distance = self.params.corner_cutting + (
            self.params.straight_smoothing * self.last_speed
        )

        predicted_car_position = Point(0.0, prediction_distance)
        left_point = left_circle.get_closest_point(predicted_car_position)
        right_point = right_circle.get_closest_point(predicted_car_position)

        target_position = Point(
            (left_point.x + right_point.x) / 2.0,
            (left_point.y + right_point.y) / 2.0,
        )
        error = (target_position.x - predicted_car_position.x) / prediction_distance
        if math.isnan(error) or math.isinf(error):
            error = 0.0

        steering_angle = self.pid.update_and_get_correction(error, delta_time)

        radius = min(left_circle.radius, right_circle.radius)
        speed_limit_radius = map_value(
            self.params.radius_lower,
            self.params.radius_upper,
            0.0,
            1.0,
            radius,
        )
        speed_limit_error = max(
            0.0,
            1.0
            + self.params.steering_slow_down_dead_zone
            - abs(error) * self.params.steering_slow_down,
        )
        speed_limit_acceleration = (
            self.last_speed + self.params.max_acceleration * delta_time
        )
        speed_limit_barrier = (
            map_value(
                self.params.barrier_lower_limit,
                self.params.barrier_upper_limit,
                0.0,
                1.0,
                barrier,
            )
            ** self.params.barrier_exponent
        )

        relative_speed = min(
            speed_limit_error,
            speed_limit_radius,
            speed_limit_acceleration,
            speed_limit_barrier,
        )
        self.last_speed = relative_speed
        speed = map_value(
            0.0,
            1.0,
            self.params.min_throttle,
            self.params.max_throttle,
            relative_speed,
        )
        steering_angle = steering_angle * map_value(
            self.params.high_speed_steering_limit_dead_zone,
            1.0,
            1.0,
            self.params.high_speed_steering_limit,
            relative_speed,
        )
        self.drive(steering_angle, speed)

        show_line_in_rviz(
            self.marker_pub,
            self.get_clock(),
            2,
            [left_point, right_point],
            color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.3),
            line_width=0.005,
        )
        show_line_in_rviz(
            self.marker_pub,
            self.get_clock(),
            3,
            [Point(0.0, 0.0), predicted_car_position],
            color=ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.3),
            line_width=0.005,
        )
        show_line_in_rviz(
            self.marker_pub,
            self.get_clock(),
            4,
            [predicted_car_position, target_position],
            color=ColorRGBA(r=1.0, g=0.4, b=0.0, a=1.0),
        )
        show_line_in_rviz(
            self.marker_pub,
            self.get_clock(),
            5,
            [Point(-2.0, barrier), Point(2.0, barrier)],
            color=ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0),
        )

    def handle_scan(self, laser_scan, delta_time):
        points = self.get_scan_as_cartesian(laser_scan)
        if points.shape[0] == 0:
            self.get_logger().warn(
                "Skipping laser scan: no finite values."
            )
            return

        split = self.find_left_right_border(points)
        right_wall = points[:split:4, :]
        left_wall = points[split::4, :]

        left_circle = Circle.fit(left_wall)
        right_circle = Circle.fit(right_wall)

        barrier_start = int(
            points.shape[0] * (0.5 - self.params.barrier_size_realtive)
        )
        barrier_end = int(
            points.shape[0] * (0.5 + self.params.barrier_size_realtive)
        )
        barrier = np.max(points[barrier_start: barrier_end, 1])

        self.follow_walls(left_circle, right_circle, barrier, delta_time)

        show_circle_in_rviz(
            self.marker_pub, self.get_clock(), left_circle, left_wall, 0
        )
        show_circle_in_rviz(
            self.marker_pub, self.get_clock(), right_circle, right_wall, 1
        )

    def _stamp_to_sec(self, stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def laser_callback(self, scan_message):
        stamp = scan_message.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            scan_time = self.get_clock().now().nanoseconds * 1e-9
        else:
            scan_time = self._stamp_to_sec(stamp)

        if (
            self.last_scan_time is not None
            and abs(scan_time - self.last_scan_time) > 0.0001
            and scan_time > self.last_scan_time
        ):
            delta_time = scan_time - self.last_scan_time
            self.handle_scan(scan_message, delta_time)

        self.last_scan_time = scan_time


def main():
    rclpy.init()
    node = WallFollowingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
