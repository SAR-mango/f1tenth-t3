import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from . import weighted_pairs_mmse_algorithm


NODE_PARAM_DEFAULTS = {
    "scan_topic": "/scan",
    "cmd_vel_topic": "/cmd_vel",
    "steering_radius_sign": 1.0,
    "straight_radius_command_m": 0.0,
    "stop_on_algorithm_fallback": True,
    "front_stop_distance_m": 0.2,
    "front_stop_half_angle_deg": 15.0,
    "log_status_interval_sec": 1.0,
}


def _coerce_node_parameter(name, value):
    expected = NODE_PARAM_DEFAULTS[name]
    if isinstance(expected, bool):
        if isinstance(value, str):
            return value.strip().lower() in ("true", "1", "yes", "on")
        return bool(value)
    if isinstance(expected, float):
        return float(value)
    if isinstance(expected, int):
        return int(value)
    if isinstance(expected, str):
        return str(value)
    return value


class WeightedPairsMmseNode(Node):
    def __init__(self):
        super().__init__("weighted_pairs_mmse")
        self._declare_parameters()
        self._apply_parameters(self._collect_parameter_values())
        self.add_on_set_parameters_callback(self._on_params)

        self._last_status_log_time = 0.0
        self._last_status_message = ""

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        scan_qos = QoSProfile(depth=10)
        scan_qos.reliability = ReliabilityPolicy.RELIABLE
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._scan_callback,
            scan_qos,
        )

        self.get_logger().info(
            "weighted_pairs_mmse ready. scan=%s cmd_vel=%s steering_radius_sign=%.1f "
            "front_stop=%.3fm inside +/-%.1fdeg "
            "(raw command convention: positive radius = left, negative = right)"
            % (
                self.scan_topic,
                self.cmd_vel_topic,
                self.steering_radius_sign,
                self.front_stop_distance_m,
                self.front_stop_half_angle_deg,
            )
        )

    def _declare_parameters(self):
        descriptor = ParameterDescriptor(dynamic_typing=True)
        for name, default in NODE_PARAM_DEFAULTS.items():
            self.declare_parameter(name, default, descriptor)
        for name, default in weighted_pairs_mmse_algorithm.PARAM_DEFAULTS.items():
            self.declare_parameter(name, default, descriptor)

    def _collect_parameter_values(self, overrides=None):
        values = {}
        for name in NODE_PARAM_DEFAULTS.keys():
            values[name] = self.get_parameter(name).value
        for name in weighted_pairs_mmse_algorithm.PARAM_DEFAULTS.keys():
            values[name] = self.get_parameter(name).value

        if overrides is not None:
            for parameter in overrides:
                values[parameter.name] = parameter.value

        return values

    def _apply_parameters(self, parameter_values):
        self.scan_topic = _coerce_node_parameter(
            "scan_topic",
            parameter_values["scan_topic"],
        )
        self.cmd_vel_topic = _coerce_node_parameter(
            "cmd_vel_topic",
            parameter_values["cmd_vel_topic"],
        )
        self.steering_radius_sign = _coerce_node_parameter(
            "steering_radius_sign",
            parameter_values["steering_radius_sign"],
        )
        self.straight_radius_command_m = _coerce_node_parameter(
            "straight_radius_command_m",
            parameter_values["straight_radius_command_m"],
        )
        self.stop_on_algorithm_fallback = _coerce_node_parameter(
            "stop_on_algorithm_fallback",
            parameter_values["stop_on_algorithm_fallback"],
        )
        self.front_stop_distance_m = _coerce_node_parameter(
            "front_stop_distance_m",
            parameter_values["front_stop_distance_m"],
        )
        self.front_stop_half_angle_deg = _coerce_node_parameter(
            "front_stop_half_angle_deg",
            parameter_values["front_stop_half_angle_deg"],
        )
        self.log_status_interval_sec = _coerce_node_parameter(
            "log_status_interval_sec",
            parameter_values["log_status_interval_sec"],
        )

        weighted_pairs_mmse_algorithm.configure_parameters(
            {
                name: parameter_values[name]
                for name in weighted_pairs_mmse_algorithm.PARAM_DEFAULTS.keys()
            }
        )

        if self.log_status_interval_sec < 0.0:
            raise ValueError("log_status_interval_sec must be >= 0")
        if self.front_stop_half_angle_deg < 0.0:
            raise ValueError("front_stop_half_angle_deg must be >= 0")

    def _on_params(self, params):
        previous_scan_topic = getattr(self, "scan_topic", None)
        previous_cmd_vel_topic = getattr(self, "cmd_vel_topic", None)

        try:
            self._apply_parameters(self._collect_parameter_values(params))
        except Exception as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        if previous_scan_topic is not None and self.scan_topic != previous_scan_topic:
            self.get_logger().warn("scan_topic changes require a node restart to take effect.")
        if previous_cmd_vel_topic is not None and self.cmd_vel_topic != previous_cmd_vel_topic:
            self.get_logger().warn("cmd_vel_topic changes require a node restart to take effect.")

        return SetParametersResult(successful=True)

    def _scan_to_algorithm_input(self, scan: LaserScan):
        if not scan.ranges:
            return {
                "angles_deg": np.zeros(0, dtype=np.float64),
                "ranges_m": np.zeros(0, dtype=np.float64),
                "hit_valid": np.zeros(0, dtype=bool),
            }

        ranges_m = np.asarray(scan.ranges, dtype=np.float64)
        angles_rad = scan.angle_min + np.arange(ranges_m.shape[0], dtype=np.float64) * scan.angle_increment

        hit_valid = np.isfinite(ranges_m) & (ranges_m > 0.0)
        if scan.range_min > 0.0:
            hit_valid &= ranges_m >= scan.range_min
        if math.isfinite(scan.range_max) and scan.range_max > 0.0:
            hit_valid &= ranges_m <= scan.range_max

        return {
            "angles_deg": np.rad2deg(angles_rad),
            "ranges_m": ranges_m,
            "hit_valid": hit_valid,
        }

    def _radius_to_command(self, steering_radius_m: float) -> float:
        if not math.isfinite(steering_radius_m):
            return self.straight_radius_command_m
        return self.steering_radius_sign * float(steering_radius_m)

    def _extract_front_min_distance(self, scan: LaserScan) -> float:
        if not scan.ranges or self.front_stop_distance_m <= 0.0:
            return math.inf

        half_angle_rad = math.radians(max(self.front_stop_half_angle_deg, 0.0))
        ranges_m = np.asarray(scan.ranges, dtype=np.float64)
        angles_rad = scan.angle_min + np.arange(ranges_m.shape[0], dtype=np.float64) * scan.angle_increment

        front_mask = np.abs(angles_rad) <= half_angle_rad
        if not np.any(front_mask):
            return math.inf

        front_ranges_m = ranges_m[front_mask]
        valid_front_hits = np.isfinite(front_ranges_m) & (front_ranges_m >= 0.0)
        if not np.any(valid_front_hits):
            return math.inf

        return float(np.min(front_ranges_m[valid_front_hits]))

    def _publish_cmd(self, speed_mps: float, steering_radius_m: float):
        msg = Twist()
        msg.linear.x = float(speed_mps)
        msg.angular.z = self._radius_to_command(steering_radius_m)
        self.cmd_pub.publish(msg)

    def _maybe_log_status(self, status: str):
        now_sec = time.monotonic()
        if (
            status != self._last_status_message
            or now_sec - self._last_status_log_time >= self.log_status_interval_sec
        ):
            self.get_logger().info(status)
            self._last_status_log_time = now_sec
            self._last_status_message = status

    def _scan_callback(self, scan: LaserScan):
        front_min_distance_m = self._extract_front_min_distance(scan)
        if front_min_distance_m <= self.front_stop_distance_m:
            self._publish_cmd(0.0, math.inf)
            self._maybe_log_status(
                "front stop | min=%.3f m <= %.3f m inside +/-%.1f deg | commanded stop"
                % (
                    front_min_distance_m,
                    self.front_stop_distance_m,
                    self.front_stop_half_angle_deg,
                )
            )
            return

        try:
            result = weighted_pairs_mmse_algorithm.runAutonomousAlgorithm(
                self._scan_to_algorithm_input(scan)
            )
        except Exception as exc:
            self.get_logger().error(f"weighted_pairs_mmse failed; publishing stop command: {exc}")
            self._publish_cmd(0.0, math.inf)
            return

        status = str(result.get("status", "weighted pairs ok"))
        speed_mps = float(result.get("speed_mps", 0.0))
        steering_radius_m = float(result.get("steering_radius_m", math.inf))
        if not math.isfinite(speed_mps):
            speed_mps = 0.0
        if self.stop_on_algorithm_fallback and "fallback" in status.lower():
            speed_mps = 0.0
            steering_radius_m = math.inf
            status = f"{status} | commanded stop"

        self._publish_cmd(speed_mps, steering_radius_m)
        self._maybe_log_status(status)

    def destroy_node(self):
        self._publish_cmd(0.0, math.inf)
        super().destroy_node()


def main():
    rclpy.init()
    node = WeightedPairsMmseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
