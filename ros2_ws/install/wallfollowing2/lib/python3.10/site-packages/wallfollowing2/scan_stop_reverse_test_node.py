import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanStopReverseTestNode(Node):
    """Drive forward until obstacle threshold, then reverse a fixed distance."""

    def __init__(self):
        super().__init__("scan_stop_reverse_test")

        self.startup_delay_sec = float(self.declare_parameter("startup_delay_sec", 6.0).value)
        self.forward_speed_mps = float(self.declare_parameter("forward_speed_mps", 0.6).value)
        self.stop_distance_m = float(self.declare_parameter("stop_distance_m", 0.5).value)
        self.reverse_speed_mps = float(self.declare_parameter("reverse_speed_mps", 0.3).value)
        self.reverse_distance_m = float(self.declare_parameter("reverse_distance_m", 0.5).value)
        self.stop_hold_sec = float(self.declare_parameter("stop_hold_sec", 3.0).value)
        self.front_window_deg = float(self.declare_parameter("front_window_deg", 2.0).value)
        self.control_rate_hz = float(self.declare_parameter("control_rate_hz", 20.0).value)
        self.scan_topic = str(self.declare_parameter("scan_topic", "/scan").value)
        self.cmd_vel_topic = str(self.declare_parameter("cmd_vel_topic", "/cmd_vel").value)

        self._boot_wall_time = time.monotonic()
        self._state = "WAIT_DELAY"
        self._state_start_wall_time = self._boot_wall_time
        self._front_distance_m = math.inf
        self._has_scan = False
        self._warned_no_scan = False
        self._reverse_duration_sec = self._compute_reverse_duration()

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        scan_qos = QoSProfile(depth=10)
        scan_qos.reliability = ReliabilityPolicy.RELIABLE
        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._scan_callback,
            scan_qos,
        )

        self.timer = self.create_timer(1.0 / max(self.control_rate_hz, 1.0), self._control_loop)

        self.get_logger().info(
            "Test node ready: wait %.2fs, forward %.2fm/s until front <= %.2fm, "
            "hold stop %.2fs, reverse %.2fm at %.2fm/s, then stop."
            % (
                self.startup_delay_sec,
                self.forward_speed_mps,
                self.stop_distance_m,
                self.stop_hold_sec,
                self.reverse_distance_m,
                self.reverse_speed_mps,
            )
        )

    def _compute_reverse_duration(self) -> float:
        speed = abs(self.reverse_speed_mps)
        if speed < 1e-6:
            return 0.0
        return max(0.0, self.reverse_distance_m / speed)

    def _scan_callback(self, scan: LaserScan):
        self._has_scan = True
        self._front_distance_m = self._extract_front_distance(scan)

    def _extract_front_distance(self, scan: LaserScan) -> float:
        if not scan.ranges or scan.angle_increment == 0.0:
            return math.inf

        center_index = int(round((0.0 - scan.angle_min) / scan.angle_increment))
        center_index = max(0, min(len(scan.ranges) - 1, center_index))

        half_window_rad = math.radians(max(self.front_window_deg, 0.0))
        half_window_samples = int(round(half_window_rad / abs(scan.angle_increment)))

        i_min = max(0, center_index - half_window_samples)
        i_max = min(len(scan.ranges) - 1, center_index + half_window_samples)

        finite = []
        for r in scan.ranges[i_min : i_max + 1]:
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                finite.append(r)

        if not finite:
            return math.inf

        return min(finite)

    def _publish_cmd(self, linear_x: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def _set_state(self, new_state: str, log: str):
        if self._state == new_state:
            return
        self._state = new_state
        self._state_start_wall_time = time.monotonic()
        self.get_logger().info(log)

    def _elapsed_state(self) -> float:
        return time.monotonic() - self._state_start_wall_time

    def _control_loop(self):
        now_since_boot = time.monotonic() - self._boot_wall_time

        if self._state == "WAIT_DELAY":
            self._publish_cmd(0.0)
            if now_since_boot >= self.startup_delay_sec:
                self._set_state("FORWARD", "Startup delay elapsed. Driving forward.")
            return

        if self._state == "FORWARD":
            self._publish_cmd(self.forward_speed_mps)
            if not self._has_scan and not self._warned_no_scan:
                self.get_logger().warn("No /scan data yet; continuing forward command.")
                self._warned_no_scan = True
            if self._has_scan and self._front_distance_m <= self.stop_distance_m:
                self._set_state(
                    "STOP_BEFORE_REVERSE",
                    (
                        "Front distance %.3fm <= %.3fm. Stopping before reverse."
                        % (self._front_distance_m, self.stop_distance_m)
                    ),
                )
            return

        if self._state == "STOP_BEFORE_REVERSE":
            self._publish_cmd(0.0)
            if self._elapsed_state() >= self.stop_hold_sec:
                if self._reverse_duration_sec <= 0.0:
                    self._set_state("DONE", "Reverse duration is zero. Staying stopped.")
                else:
                    self._set_state(
                        "REVERSE",
                        (
                            "Reversing for %.2fs to cover %.2fm."
                            % (self._reverse_duration_sec, self.reverse_distance_m)
                        ),
                    )
            return

        if self._state == "REVERSE":
            self._publish_cmd(-abs(self.reverse_speed_mps))
            if self._elapsed_state() >= self._reverse_duration_sec:
                self._set_state("DONE", "Reverse target reached. Remaining stopped.")
            return

        self._publish_cmd(0.0)

    def destroy_node(self):
        self._publish_cmd(0.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = ScanStopReverseTestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
