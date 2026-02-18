import math
from types import SimpleNamespace

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from drive_msgs.msg import DriveParam


PARAM_DEFAULTS = {
    "left_distance_setpoint": 0.25,
    "lookahead_distance": 0.70,
    "kp": 4.2,
    "kd": 0.75,
    "steering_limit": 1.0,
    "steering_sign": -1.0,
    "min_throttle": 0.55,
    "max_throttle": 1.0,
    "steering_speed_reduction": 0.35,
    "max_acceleration": 2.5,
    "front_stop_distance": 0.18,
    "front_slow_distance": 0.55,
    "front_angle_half_width_deg": 10.0,
    "usable_laser_range": 220.0,
    "wall_sample_min_x": 0.05,
    "wall_sample_max_x": 1.10,
    "wall_sample_min_y": 0.10,
    "wall_sample_max_y": 1.40,
    "sample_stride": 2,
}

TOPIC_DRIVE_PARAMETERS = "/input/drive_param/autonomous"
TOPIC_LASER_SCAN = "/scan"


def clamp(value, lower, upper):
    return min(upper, max(lower, value))


def coerce_parameter_value(name, value):
    expected = PARAM_DEFAULTS[name]
    if isinstance(expected, float):
        return float(value)
    if isinstance(expected, int):
        return int(value)
    if isinstance(expected, bool):
        return bool(value)
    return value


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

        self.last_scan_time = None
        self.last_error = 0.0
        self.last_speed = 0.0

    def _declare_parameters(self):
        dynamic_descriptor = ParameterDescriptor(dynamic_typing=True)
        for name, default in PARAM_DEFAULTS.items():
            self.declare_parameter(name, default, dynamic_descriptor)

    def _load_parameters(self):
        for name in PARAM_DEFAULTS.keys():
            value = self.get_parameter(name).value
            setattr(self.params, name, coerce_parameter_value(name, value))

    def _on_params(self, _params):
        self._load_parameters()
        return SetParametersResult(successful=True)

    def drive(self, angle, velocity):
        message = DriveParam()
        message.angle = float(angle)
        message.velocity = float(velocity)
        self.drive_pub.publish(message)

    def _scan_to_arrays(self, laser_scan):
        ranges = np.array(laser_scan.ranges, dtype=np.float64)
        if ranges.shape[0] == 0:
            return np.zeros(0), np.zeros(0)

        angles = np.linspace(
            laser_scan.angle_min,
            laser_scan.angle_max,
            ranges.shape[0],
            dtype=np.float64,
        )

        laser_range = laser_scan.angle_max - laser_scan.angle_min
        usable_range = math.radians(self.params.usable_laser_range)
        if usable_range < laser_range:
            skip_left = int(
                (-laser_scan.angle_min - usable_range / 2.0)
                / laser_range
                * ranges.shape[0]
            )
            skip_right = int(
                (laser_scan.angle_max - usable_range / 2.0)
                / laser_range
                * ranges.shape[0]
            )
            end = -1 - skip_right
            angles = angles[skip_left:end]
            ranges = ranges[skip_left:end]

        finite = np.isfinite(ranges)
        positive = ranges > 0.0
        mask = finite & positive
        return angles[mask], ranges[mask]

    def _to_cartesian(self, angles, ranges):
        points = np.zeros((ranges.shape[0], 2), dtype=np.float64)
        # Same frame convention used by the previous controller:
        # +x left, +y forward.
        points[:, 0] = -np.sin(angles) * ranges
        points[:, 1] = np.cos(angles) * ranges
        return points

    def _front_min(self, angles, ranges):
        half = math.radians(self.params.front_angle_half_width_deg)
        mask = np.abs(angles) <= half
        if not np.any(mask):
            return float("inf")
        front_ranges = ranges[mask]
        if front_ranges.shape[0] == 0:
            return float("inf")
        return float(np.min(front_ranges))

    def _estimate_left_distance(self, points):
        mask = (
            (points[:, 0] >= self.params.wall_sample_min_x)
            & (points[:, 0] <= self.params.wall_sample_max_x)
            & (points[:, 1] >= self.params.wall_sample_min_y)
            & (points[:, 1] <= self.params.wall_sample_max_y)
        )
        wall_points = points[mask]
        if wall_points.shape[0] < 4:
            raise ValueError("insufficient left-wall points")

        stride = max(1, int(self.params.sample_stride))
        wall_points = wall_points[::stride]
        if wall_points.shape[0] < 3:
            raise ValueError("insufficient left-wall points after subsampling")

        y = wall_points[:, 1]
        x = wall_points[:, 0]
        slope, intercept = np.polyfit(y, x, 1)

        lookahead = max(0.05, float(self.params.lookahead_distance))
        left_distance = slope * lookahead + intercept
        if not np.isfinite(left_distance):
            raise ValueError("left distance estimate is not finite")
        return float(left_distance)

    def _speed_command(self, steering_magnitude, front_min, delta_time):
        desired_speed = (
            self.params.max_throttle
            - self.params.steering_speed_reduction * steering_magnitude
        )
        desired_speed = clamp(
            desired_speed, self.params.min_throttle, self.params.max_throttle
        )

        if front_min <= self.params.front_stop_distance:
            return 0.0

        if front_min < self.params.front_slow_distance:
            span = max(
                1e-6,
                self.params.front_slow_distance - self.params.front_stop_distance,
            )
            front_scale = clamp(
                (front_min - self.params.front_stop_distance) / span,
                0.0,
                1.0,
            )
            desired_speed *= max(0.4, front_scale)

        max_delta = max(0.0, self.params.max_acceleration) * delta_time
        lower = self.last_speed - max_delta
        upper = self.last_speed + max_delta
        return clamp(desired_speed, lower, upper)

    def handle_scan(self, laser_scan, delta_time):
        angles, ranges = self._scan_to_arrays(laser_scan)
        if ranges.shape[0] == 0:
            self.get_logger().warn("Skipping scan: empty/invalid range set.")
            return

        front_min = self._front_min(angles, ranges)
        if front_min <= self.params.front_stop_distance:
            self.last_speed = 0.0
            self.drive(0.0, 0.0)
            return

        points = self._to_cartesian(angles, ranges)
        try:
            left_distance = self._estimate_left_distance(points)
        except ValueError as exc:
            self.get_logger().warn(f"Skipping scan: {exc}")
            return

        error = left_distance - self.params.left_distance_setpoint
        derivative = (error - self.last_error) / max(1e-6, delta_time)
        self.last_error = error

        steering = self.params.kp * error + self.params.kd * derivative
        steering = clamp(
            steering,
            -float(self.params.steering_limit),
            float(self.params.steering_limit),
        )
        steering = clamp(
            steering * float(self.params.steering_sign),
            -1.0,
            1.0,
        )

        speed = self._speed_command(abs(steering), front_min, delta_time)
        self.last_speed = speed
        self.drive(steering, speed)

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
