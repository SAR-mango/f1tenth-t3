import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from drive_msgs.msg import DriveParam


class FollowTheGapNode(Node):
    def __init__(self):
        super().__init__("follow_the_gap")

        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/input/drive_param/autonomous")
        self.declare_parameter("field_of_view_degrees", 220.0)
        self.declare_parameter("bubble_angle_degrees", 18.0)
        self.declare_parameter("min_range", 0.05)
        self.declare_parameter("max_range", 8.0)
        self.declare_parameter("min_gap_points", 10)
        self.declare_parameter("front_stop_distance", 0.45)
        self.declare_parameter("front_window_degrees", 12.0)
        self.declare_parameter("min_speed", 0.12)
        self.declare_parameter("max_speed", 0.45)

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.drive_topic = str(self.get_parameter("drive_topic").value)
        self.field_of_view_degrees = float(self.get_parameter("field_of_view_degrees").value)
        self.bubble_angle_degrees = float(self.get_parameter("bubble_angle_degrees").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.min_gap_points = int(self.get_parameter("min_gap_points").value)
        self.front_stop_distance = float(self.get_parameter("front_stop_distance").value)
        self.front_window_degrees = float(self.get_parameter("front_window_degrees").value)
        self.min_speed = float(self.get_parameter("min_speed").value)
        self.max_speed = float(self.get_parameter("max_speed").value)

        self.drive_pub = self.create_publisher(DriveParam, self.drive_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        self.get_logger().info(
            f"follow_the_gap started: scan={self.scan_topic}, drive={self.drive_topic}"
        )

    def _publish_drive(self, angle, speed):
        message = DriveParam()
        message.angle = float(max(-1.0, min(1.0, angle)))
        message.velocity = float(max(0.0, speed))
        self.drive_pub.publish(message)

    def _largest_gap(self, ranges):
        valid = ranges > 0.0
        best_start = -1
        best_end = -1
        best_length = 0

        current_start = -1
        for index, is_valid in enumerate(valid):
            if is_valid and current_start < 0:
                current_start = index
            elif not is_valid and current_start >= 0:
                current_length = index - current_start
                if current_length > best_length:
                    best_length = current_length
                    best_start = current_start
                    best_end = index
                current_start = -1

        if current_start >= 0:
            current_length = len(valid) - current_start
            if current_length > best_length:
                best_length = current_length
                best_start = current_start
                best_end = len(valid)

        return best_start, best_end, best_length

    def scan_callback(self, scan):
        if len(scan.ranges) == 0:
            self._publish_drive(0.0, 0.0)
            return

        ranges = np.array(scan.ranges, dtype=np.float32)
        finite_mask = np.isfinite(ranges)
        ranges[~finite_mask] = self.max_range
        ranges = np.clip(ranges, self.min_range, self.max_range)

        angles = scan.angle_min + np.arange(ranges.shape[0], dtype=np.float32) * scan.angle_increment

        half_fov = math.radians(self.field_of_view_degrees) * 0.5
        in_fov = np.abs(angles) <= half_fov
        if not np.any(in_fov):
            self._publish_drive(0.0, 0.0)
            return

        fov_ranges = ranges[in_fov]
        fov_angles = angles[in_fov]

        closest_index = int(np.argmin(fov_ranges))
        bubble_half_width = max(
            1,
            int(
                math.radians(self.bubble_angle_degrees)
                / max(abs(scan.angle_increment), 1e-6)
            ),
        )
        bubble_start = max(0, closest_index - bubble_half_width)
        bubble_end = min(fov_ranges.shape[0], closest_index + bubble_half_width + 1)
        fov_ranges[bubble_start:bubble_end] = 0.0

        gap_start, gap_end, gap_length = self._largest_gap(fov_ranges)
        if gap_start < 0 or gap_end <= gap_start or gap_length < self.min_gap_points:
            self._publish_drive(0.0, 0.0)
            return

        gap_ranges = fov_ranges[gap_start:gap_end]
        target_index = gap_start + int(np.argmax(gap_ranges))
        target_angle = float(fov_angles[target_index])

        steering = target_angle / max(half_fov, 1e-6)
        steering = max(-1.0, min(1.0, steering))

        front_window = math.radians(self.front_window_degrees) * 0.5
        front_mask = np.abs(fov_angles) <= front_window
        if np.any(front_mask):
            front_distance = float(np.min(fov_ranges[front_mask]))
            if front_distance <= self.front_stop_distance:
                self._publish_drive(steering, 0.0)
                return

        speed_scale = 1.0 - min(1.0, abs(steering))
        speed = self.min_speed + (self.max_speed - self.min_speed) * speed_scale
        self._publish_drive(steering, speed)


def main():
    rclpy.init()
    node = FollowTheGapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
