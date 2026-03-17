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
        self.declare_parameter("bubble_turn_gain_degrees", 16.0)
        self.declare_parameter("side_clearance_min", 0.35)
        self.declare_parameter("wall_bias_gain", 0.25)
        self.declare_parameter("wall_bias_deadband_m", 0.12)
        self.declare_parameter("steering_smoothing_alpha", 0.72)
        self.declare_parameter("max_steering_step", 0.10)
        self.declare_parameter("turn_intent_weight", 0.0)
        self.declare_parameter("sharp_turn_steering_threshold", 0.55)
        self.declare_parameter("sharp_turn_extra_steering_step", 0.0)
        self.declare_parameter("sharp_turn_alpha_reduction", 0.0)
        self.declare_parameter("corner_escape_turn_threshold_degrees", 0.0)
        self.declare_parameter("corner_escape_front_distance", 0.0)
        self.declare_parameter("corner_escape_wall_margin", 0.0)
        self.declare_parameter("corner_escape_bias", 0.0)
        self.declare_parameter("corner_escape_extra_steering_step", 0.0)
        self.declare_parameter("corner_escape_alpha_reduction", 0.0)
        self.declare_parameter("side_speed_floor", 0.25)
        self.declare_parameter("min_range", 0.05)
        self.declare_parameter("max_range", 8.0)
        self.declare_parameter("min_gap_points", 10)
        self.declare_parameter("hard_stop_distance", 0.24)
        self.declare_parameter("front_stop_distance", 0.45)
        self.declare_parameter("front_caution_distance", 1.35)
        self.declare_parameter("front_window_degrees", 12.0)
        self.declare_parameter("front_path_window_degrees", 10.0)
        self.declare_parameter("front_path_steering_threshold_degrees", 18.0)
        self.declare_parameter("front_stop_percentile", 12.0)
        self.declare_parameter("front_stop_hold_frames", 3)
        self.declare_parameter("steering_slowdown_exponent", 1.0)
        self.declare_parameter("min_speed", 0.12)
        self.declare_parameter("max_speed", 0.45)

        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.drive_topic = str(self.get_parameter("drive_topic").value)
        self.field_of_view_degrees = float(self.get_parameter("field_of_view_degrees").value)
        self.bubble_angle_degrees = float(self.get_parameter("bubble_angle_degrees").value)
        self.bubble_turn_gain_degrees = float(
            self.get_parameter("bubble_turn_gain_degrees").value
        )
        self.side_clearance_min = float(self.get_parameter("side_clearance_min").value)
        self.wall_bias_gain = float(self.get_parameter("wall_bias_gain").value)
        self.wall_bias_deadband_m = float(self.get_parameter("wall_bias_deadband_m").value)
        self.steering_smoothing_alpha = float(
            self.get_parameter("steering_smoothing_alpha").value
        )
        self.max_steering_step = float(self.get_parameter("max_steering_step").value)
        self.turn_intent_weight = float(self.get_parameter("turn_intent_weight").value)
        self.sharp_turn_steering_threshold = float(
            self.get_parameter("sharp_turn_steering_threshold").value
        )
        self.sharp_turn_extra_steering_step = float(
            self.get_parameter("sharp_turn_extra_steering_step").value
        )
        self.sharp_turn_alpha_reduction = float(
            self.get_parameter("sharp_turn_alpha_reduction").value
        )
        self.corner_escape_turn_threshold_degrees = float(
            self.get_parameter("corner_escape_turn_threshold_degrees").value
        )
        self.corner_escape_front_distance = float(
            self.get_parameter("corner_escape_front_distance").value
        )
        self.corner_escape_wall_margin = float(
            self.get_parameter("corner_escape_wall_margin").value
        )
        self.corner_escape_bias = float(self.get_parameter("corner_escape_bias").value)
        self.corner_escape_extra_steering_step = float(
            self.get_parameter("corner_escape_extra_steering_step").value
        )
        self.corner_escape_alpha_reduction = float(
            self.get_parameter("corner_escape_alpha_reduction").value
        )
        self.side_speed_floor = float(self.get_parameter("side_speed_floor").value)
        self.min_range = float(self.get_parameter("min_range").value)
        self.max_range = float(self.get_parameter("max_range").value)
        self.min_gap_points = int(self.get_parameter("min_gap_points").value)
        self.hard_stop_distance = float(self.get_parameter("hard_stop_distance").value)
        self.front_stop_distance = float(self.get_parameter("front_stop_distance").value)
        self.front_caution_distance = float(
            self.get_parameter("front_caution_distance").value
        )
        self.front_window_degrees = float(self.get_parameter("front_window_degrees").value)
        self.front_path_window_degrees = float(
            self.get_parameter("front_path_window_degrees").value
        )
        self.front_path_steering_threshold_degrees = float(
            self.get_parameter("front_path_steering_threshold_degrees").value
        )
        self.front_stop_percentile = float(
            self.get_parameter("front_stop_percentile").value
        )
        self.front_stop_hold_frames = int(
            self.get_parameter("front_stop_hold_frames").value
        )
        self.steering_slowdown_exponent = float(
            self.get_parameter("steering_slowdown_exponent").value
        )
        self.min_speed = float(self.get_parameter("min_speed").value)
        self.max_speed = float(self.get_parameter("max_speed").value)
        self.last_steering = 0.0
        self.front_stop_counter = 0

        self.drive_pub = self.create_publisher(DriveParam, self.drive_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        self.get_logger().info(
            f"follow_the_gap started: scan={self.scan_topic}, drive={self.drive_topic}"
        )

    def _publish_drive(self, angle, speed):
        message = DriveParam()
        self.last_steering = float(max(-1.0, min(1.0, angle)))
        message.angle = self.last_steering
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
        # Treat non-finite and non-physical near-zero returns as "no obstacle"
        valid_mask = np.isfinite(ranges) & (ranges > max(self.min_range, 1e-3))
        ranges[~valid_mask] = self.max_range
        ranges = np.clip(ranges, self.min_range, self.max_range)

        angles = scan.angle_min + np.arange(ranges.shape[0], dtype=np.float32) * scan.angle_increment

        half_fov = math.radians(self.field_of_view_degrees) * 0.5
        in_fov = np.abs(angles) <= half_fov
        if not np.any(in_fov):
            self._publish_drive(0.0, 0.0)
            return

        fov_ranges = ranges[in_fov].copy()
        fov_ranges_raw = fov_ranges.copy()
        fov_angles = angles[in_fov]

        closest_index = int(np.argmin(fov_ranges))
        closest_distance = float(fov_ranges[closest_index])
        adaptive_bubble_degrees = self.bubble_angle_degrees + (
            self.bubble_turn_gain_degrees * min(1.0, abs(self.last_steering))
        )
        clearance_bubble_degrees = math.degrees(
            math.atan2(self.side_clearance_min, max(closest_distance, self.min_range))
        )
        effective_bubble_degrees = max(adaptive_bubble_degrees, clearance_bubble_degrees)
        bubble_half_width = max(
            1,
            int(
                math.radians(effective_bubble_degrees)
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
        gap_len = gap_ranges.shape[0]
        center_weights = 1.0 - 0.15 * np.abs(np.linspace(-1.0, 1.0, gap_len, dtype=np.float32))
        if gap_len >= 9:
            smooth_kernel = np.ones(9, dtype=np.float32) / 9.0
            smoothed = np.convolve(gap_ranges, smooth_kernel, mode="same")
        elif gap_len >= 3:
            smooth_kernel = np.ones(3, dtype=np.float32) / 3.0
            smoothed = np.convolve(gap_ranges, smooth_kernel, mode="same")
        else:
            smoothed = gap_ranges

        scores = smoothed * center_weights
        safe_mask = gap_ranges >= self.side_clearance_min
        if np.any(safe_mask):
            scores = np.where(safe_mask, scores, -1.0)
        target_local = int(np.argmax(scores))
        target_index = gap_start + target_local
        target_angle = float(fov_angles[target_index])

        desired_steering = target_angle / max(half_fov, 1e-6)
        desired_steering = max(-1.0, min(1.0, desired_steering))
        turn_intent = max(abs(desired_steering), min(1.0, abs(target_angle) / max(half_fov, 1e-6)))

        side_window = math.radians(70.0)
        left_mask = (fov_angles > 0.0) & (fov_angles <= side_window)
        right_mask = (fov_angles < 0.0) & (fov_angles >= -side_window)
        left_distance = float(np.min(fov_ranges_raw[left_mask])) if np.any(left_mask) else self.max_range
        right_distance = (
            float(np.min(fov_ranges_raw[right_mask])) if np.any(right_mask) else self.max_range
        )
        front_preview_window = math.radians(self.front_window_degrees) * 0.5
        front_preview_mask = np.abs(fov_angles) <= front_preview_window
        corner_front_distance = (
            float(np.min(fov_ranges_raw[front_preview_mask]))
            if np.any(front_preview_mask)
            else self.max_range
        )
        clearance_norm = max(self.side_clearance_min, 1e-6)
        side_delta = left_distance - right_distance
        if abs(side_delta) < self.wall_bias_deadband_m:
            wall_bias = 0.0
        else:
            wall_bias = self.wall_bias_gain * max(
                -1.0, min(1.0, side_delta / clearance_norm)
            )

        desired_steering = max(-1.0, min(1.0, desired_steering + wall_bias))
        corner_turn_threshold = math.radians(
            max(0.0, self.corner_escape_turn_threshold_degrees)
        )
        corner_escape_progress = 0.0
        turning_left = target_angle >= 0.0
        if (
            self.corner_escape_bias > 0.0
            and self.corner_escape_front_distance > 0.0
            and abs(target_angle) >= corner_turn_threshold
        ):
            turn_side_distance = left_distance if turning_left else right_distance
            outer_side_distance = right_distance if turning_left else left_distance
            inside_wall_gap = outer_side_distance - turn_side_distance
            if (
                corner_front_distance <= self.corner_escape_front_distance
                and inside_wall_gap >= self.corner_escape_wall_margin
            ):
                front_term = min(
                    1.0,
                    (
                        self.corner_escape_front_distance - corner_front_distance
                    )
                    / max(self.corner_escape_front_distance, 1e-6),
                )
                wall_term = min(
                    1.0,
                    (inside_wall_gap - self.corner_escape_wall_margin)
                    / max(clearance_norm, 1e-6),
                )
                corner_escape_progress = front_term * wall_term
                escape_bias = self.corner_escape_bias * corner_escape_progress
                desired_steering += -escape_bias if turning_left else escape_bias
                desired_steering = max(-1.0, min(1.0, desired_steering))
        turn_intent = max(turn_intent, abs(desired_steering))
        desired_turn_mag = abs(desired_steering)
        sharp_turn_threshold = max(0.0, min(0.95, self.sharp_turn_steering_threshold))
        sharp_turn_progress = 0.0
        if desired_turn_mag > sharp_turn_threshold:
            sharp_turn_progress = min(
                1.0,
                (desired_turn_mag - sharp_turn_threshold) / max(1e-6, 1.0 - sharp_turn_threshold),
            )

        alpha = max(
            0.0,
            min(
                0.98,
                self.steering_smoothing_alpha
                - sharp_turn_progress * max(0.0, self.sharp_turn_alpha_reduction),
                - corner_escape_progress * max(0.0, self.corner_escape_alpha_reduction),
            ),
        )
        smoothed_steering = alpha * self.last_steering + (1.0 - alpha) * desired_steering
        step = max(
            0.01,
            self.max_steering_step
            + sharp_turn_progress * max(0.0, self.sharp_turn_extra_steering_step),
            + corner_escape_progress * max(0.0, self.corner_escape_extra_steering_step),
        )
        delta = smoothed_steering - self.last_steering
        if delta > step:
            delta = step
        elif delta < -step:
            delta = -step
        steering = max(-1.0, min(1.0, self.last_steering + delta))

        front_window = math.radians(self.front_window_degrees) * 0.5
        front_mask = np.abs(fov_angles) <= front_window
        front_distance = self.max_range
        effective_front_distance = self.max_range
        effective_front_min_distance = self.max_range
        if np.any(front_mask):
            front_values = fov_ranges_raw[front_mask]
            front_min_distance = float(np.min(front_values))
            percentile = max(0.0, min(100.0, self.front_stop_percentile))
            front_distance = float(np.percentile(front_values, percentile))
            effective_front_distance = front_distance
            effective_front_min_distance = front_min_distance

            path_window = math.radians(self.front_path_window_degrees) * 0.5
            path_mask = np.abs(fov_angles - target_angle) <= path_window
            if np.any(path_mask):
                path_values = fov_ranges_raw[path_mask]
                path_min_distance = float(np.min(path_values))
                path_distance = float(np.percentile(path_values, percentile))
                threshold = math.radians(self.front_path_steering_threshold_degrees)
                if abs(target_angle) >= threshold:
                    effective_front_distance = path_distance
                    effective_front_min_distance = path_min_distance

            if effective_front_min_distance <= self.hard_stop_distance:
                self.front_stop_counter = max(1, self.front_stop_hold_frames)
                self._publish_drive(steering, 0.0)
                return

            if effective_front_distance <= self.front_stop_distance:
                self.front_stop_counter += 1
            else:
                self.front_stop_counter = 0
            if self.front_stop_counter >= max(1, self.front_stop_hold_frames):
                self._publish_drive(steering, 0.0)
                return
        else:
            self.front_stop_counter = 0

        turn_mag = min(1.0, abs(steering))
        turn_intent_weight = max(0.0, min(1.0, self.turn_intent_weight))
        speed_turn_mag = max(
            turn_mag,
            (1.0 - turn_intent_weight) * turn_mag + turn_intent_weight * turn_intent,
        )
        turn_speed_scale = 1.0 - (speed_turn_mag ** self.steering_slowdown_exponent)
        speed_scale = turn_speed_scale
        min_side_distance = min(left_distance, right_distance)
        side_clearance_scale = min(1.0, min_side_distance / clearance_norm)
        speed_scale *= max(self.side_speed_floor, side_clearance_scale)
        caution_distance = max(self.front_stop_distance + 0.05, self.front_caution_distance)
        front_progress = (effective_front_distance - self.front_stop_distance) / (
            caution_distance - self.front_stop_distance
        )
        front_progress = max(0.0, min(1.0, front_progress))
        front_speed_scale = 0.2 + 0.8 * (front_progress ** 1.5)
        speed_scale *= front_speed_scale
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
