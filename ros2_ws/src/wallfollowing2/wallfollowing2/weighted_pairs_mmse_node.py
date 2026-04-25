import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from .weighted_pairs_mmse_algorithm import runAutonomousAlgorithm


class WeightedPairsMmseNode(Node):
    def __init__(self):
        super().__init__("weighted_pairs_mmse")

        self.scan_topic = str(self.declare_parameter("scan_topic", "/scan").value)
        self.cmd_vel_topic = str(self.declare_parameter("cmd_vel_topic", "/cmd_vel").value)
        self.steering_radius_sign = float(
            self.declare_parameter("steering_radius_sign", 1.0).value
        )
        self.straight_radius_command_m = float(
            self.declare_parameter("straight_radius_command_m", 0.0).value
        )
        self.stop_on_algorithm_fallback = bool(
            self.declare_parameter("stop_on_algorithm_fallback", True).value
        )
        self.front_stop_distance_m = float(
            self.declare_parameter("front_stop_distance_m", 0.2).value
        )
        self.front_stop_half_angle_deg = float(
            self.declare_parameter("front_stop_half_angle_deg", 15.0).value
        )
        self.log_status_interval_sec = float(
            self.declare_parameter("log_status_interval_sec", 1.0).value
        )

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
            result = runAutonomousAlgorithm(self._scan_to_algorithm_input(scan))
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
