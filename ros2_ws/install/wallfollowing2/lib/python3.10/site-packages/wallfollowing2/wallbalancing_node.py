import math
from types import SimpleNamespace

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
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
    "barrier_size_relative": 0.1,
    "barrier_lower_limit": 1.0,
    "barrier_upper_limit": 15.0,
    "barrier_exponent": 1.4,
    "controller_p": 4.0,
    "controller_i": 0.2,
    "controller_d": 0.02,
    "usable_laser_range": 220.0,
    "steering_sign": -1.0,
    "front_hard_stop_distance": 0.22,
    "barrier_percentile": 25.0,
    "barrier_speed_weight": 0.35,
    "min_relative_speed": 0.0,
    "wall_balance_gain": 0.1,
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


def coerce_parameter_value(name, value):
    expected = PARAM_DEFAULTS[name]
    if isinstance(expected, float):
        return float(value)
    if isinstance(expected, int):
        return int(value)
    if isinstance(expected, bool):
        return bool(value)
    return value


class WallBalancingNode(Node):
    def __init__(self):
        super().__init__("wallbalancing")
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
        dynamic_descriptor = ParameterDescriptor(dynamic_typing=True)
        for name, default in PARAM_DEFAULTS.items():
            self.declare_parameter(name, default, dynamic_descriptor)

    def _load_parameters(self):
        for name in PARAM_DEFAULTS.keys():
            value = self.get_parameter(name).value
            setattr(self.params, name, coerce_parameter_value(name, value))
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
        if ranges.shape[0] == 0:
            return np.zeros((0, 2))

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

        finite_mask = np.isfinite(ranges)
        if not finite_mask.all():
            ranges = ranges[finite_mask]
            angles = angles[finite_mask]

        if ranges.shape[0] == 0:
            return np.zeros((0, 2))

        points = np.zeros((ranges.shape[0], 2))
        points[:, 0] = -np.sin(angles) * ranges
        points[:, 1] = np.cos(angles) * ranges
        return points

    def find_left_right_border(self, points, margin_relative=0.1):
        if points.shape[0] < 10:
            raise ValueError("Too few scan points to detect wall split.")
        margin = int(points.shape[0] * margin_relative)
        margin = max(1, min(margin, points.shape[0] // 3))
        relative = points[margin + 1:-margin, :] - points[margin:-margin - 1, :]
        if relative.shape[0] == 0:
            raise ValueError("Relative wall-segment array is empty.")
        distances = np.linalg.norm(relative, axis=1)
        return margin + np.argmax(distances) + 1

    def follow_walls(self, left_circle, right_circle, barrier, front_min, delta_time):
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

        left_clearance = math.hypot(left_point.x - predicted_car_position.x, left_point.y - predicted_car_position.y)
        right_clearance = math.hypot(right_point.x - predicted_car_position.x, right_point.y - predicted_car_position.y)
        clearance_sum = max(1e-6, left_clearance + right_clearance)
        # Positive when left wall is closer than right wall -> bias target to the right.
        balance_error = (right_clearance - left_clearance) / clearance_sum
        lateral_shift = self.params.wall_balance_gain * balance_error
        lateral_shift = max(-0.12, min(0.12, lateral_shift))
        target_position = Point(target_position.x + lateral_shift, target_position.y)

        error = (target_position.x - predicted_car_position.x) / prediction_distance
        if math.isnan(error) or math.isinf(error):
            error = 0.0

        steering_angle = self.pid.update_and_get_correction(error, delta_time)

        # Use mean wall curvature to avoid one noisy side forcing unnecessary slowdowns.
        radius = 0.5 * (left_circle.radius + right_circle.radius)
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
        speed_limit_barrier_raw = (
            map_value(
                self.params.barrier_lower_limit,
                self.params.barrier_upper_limit,
                0.0,
                1.0,
                barrier,
            )
            ** self.params.barrier_exponent
        )
        # Blend barrier gating so wall geometry can dominate speed selection on straights.
        barrier_weight = max(0.0, min(1.0, self.params.barrier_speed_weight))
        speed_limit_barrier = (
            (1.0 - barrier_weight) + barrier_weight * speed_limit_barrier_raw
        )

        relative_speed = min(
            speed_limit_error,
            speed_limit_radius,
            speed_limit_acceleration,
            speed_limit_barrier,
        )
        relative_speed = max(0.0, min(1.0, relative_speed))
        # Maintain forward progress when the front path is clear.
        min_relative_speed = max(0.0, min(1.0, self.params.min_relative_speed))
        speed_floor_span = max(
            1e-6, self.params.barrier_lower_limit - self.params.front_hard_stop_distance
        )
        floor_scale = (front_min - self.params.front_hard_stop_distance) / speed_floor_span
        floor_scale = max(0.0, min(1.0, floor_scale))
        relative_speed = max(relative_speed, min_relative_speed * floor_scale)
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
        steering_angle = steering_angle * self.params.steering_sign
        steering_angle = max(-1.0, min(1.0, steering_angle))
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

        try:
            split = self.find_left_right_border(points)
        except ValueError as exc:
            self.get_logger().warn(f"Skipping laser scan: {exc}")
            return

        right_wall = points[:split:4, :]
        left_wall = points[split::4, :]
        if right_wall.shape[0] < 3 or left_wall.shape[0] < 3:
            self.get_logger().warn("Skipping laser scan: insufficient wall points.")
            return

        try:
            left_circle = Circle.fit(left_wall)
            right_circle = Circle.fit(right_wall)
        except Exception as exc:
            self.get_logger().warn(f"Skipping laser scan: circle fit failed ({exc}).")
            return

        barrier_start = int(
            points.shape[0] * (0.5 - self.params.barrier_size_relative)
        )
        barrier_end = int(
            points.shape[0] * (0.5 + self.params.barrier_size_relative)
        )
        barrier_start = max(0, min(points.shape[0] - 1, barrier_start))
        barrier_end = max(barrier_start + 1, min(points.shape[0], barrier_end))
        front_values = points[barrier_start: barrier_end, 1]
        front_values = front_values[np.isfinite(front_values)]
        front_values = front_values[front_values > 0.0]
        if front_values.shape[0] == 0:
            front_min = self.params.barrier_upper_limit
            barrier = self.params.barrier_upper_limit
        else:
            front_min = float(np.min(front_values))
            percentile = max(0.0, min(100.0, self.params.barrier_percentile))
            # Use a lower percentile so small protruding notches don't over-slow the car.
            barrier = float(np.percentile(front_values, percentile))

        if front_min <= self.params.front_hard_stop_distance:
            self.last_speed = 0.0
            self.drive(0.0, 0.0)
            return

        self.follow_walls(left_circle, right_circle, barrier, front_min, delta_time)

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
    node = WallBalancingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
