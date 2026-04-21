import math
from types import SimpleNamespace

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from drive_msgs.msg import DriveParam


PARAM_DEFAULTS = {
    "scan_topic": "/scan",
    "drive_topic": "/input/drive_param/autonomous",
    "field_of_view_degrees": 220.0,
    "bubble_angle_degrees": 18.0,
    "bubble_turn_gain_degrees": 16.0,
    "bubble_max_angle_degrees": 45.0,
    "side_clearance_min": 0.35,
    "wall_bias_gain": 0.25,
    "wall_bias_deadband_m": 0.12,
    "steering_smoothing_alpha": 0.72,
    "max_steering_step": 0.10,
    "turn_intent_weight": 0.0,
    "sharp_turn_steering_threshold": 0.55,
    "sharp_turn_extra_steering_step": 0.0,
    "sharp_turn_alpha_reduction": 0.0,
    "corner_escape_turn_threshold_degrees": 0.0,
    "corner_escape_front_distance": 0.0,
    "corner_escape_wall_margin": 0.0,
    "corner_escape_bias": 0.0,
    "corner_escape_extra_steering_step": 0.0,
    "corner_escape_alpha_reduction": 0.0,
    "side_speed_floor": 0.25,
    "gap_fallback_speed_scale": 0.5,
    "min_range": 0.05,
    "max_range": 8.0,
    "min_gap_points": 10,
    "hard_stop_distance": 0.24,
    "front_stop_distance": 0.45,
    "front_caution_distance": 1.35,
    "front_window_degrees": 12.0,
    "front_path_window_degrees": 10.0,
    "front_path_steering_threshold_degrees": 18.0,
    "front_stop_percentile": 12.0,
    "front_stop_hold_frames": 3,
    "steering_target_angle_degrees": 70.0,
    "steering_slowdown_exponent": 1.0,
    "min_speed": 0.12,
    "max_speed": 0.45,
}


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
    if isinstance(expected, str):
        return str(value)
    return value


class FollowTheGapNode(Node):
    def __init__(self):
        super().__init__("follow_the_gap")
        self.params = SimpleNamespace()
        self._declare_parameters()
        self._load_parameters()
        self.add_on_set_parameters_callback(self._on_params)

        self.last_steering = 0.0
        self.front_stop_counter = 0

        self.drive_pub = self.create_publisher(DriveParam, self.params.drive_topic, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, self.params.scan_topic, self.scan_callback, 10
        )

        self.get_logger().info(
            f"follow_the_gap started: scan={self.params.scan_topic}, "
            f"drive={self.params.drive_topic}"
        )

    def _declare_parameters(self):
        descriptor = ParameterDescriptor(dynamic_typing=True)
        for name, default in PARAM_DEFAULTS.items():
            self.declare_parameter(name, default, descriptor)

    def _load_parameters(self):
        for name in PARAM_DEFAULTS.keys():
            value = self.get_parameter(name).value
            setattr(self.params, name, coerce_parameter_value(name, value))

    def _on_params(self, _params):
        previous_scan_topic = getattr(self.params, "scan_topic", None)
        previous_drive_topic = getattr(self.params, "drive_topic", None)
        self._load_parameters()

        if previous_scan_topic is not None and self.params.scan_topic != previous_scan_topic:
            self.get_logger().warn("scan_topic changes require a node restart to take effect.")
        if previous_drive_topic is not None and self.params.drive_topic != previous_drive_topic:
            self.get_logger().warn("drive_topic changes require a node restart to take effect.")

        return SetParametersResult(successful=True)

    def _publish_drive(self, angle, speed):
        message = DriveParam()
        self.last_steering = float(clamp(angle, -1.0, 1.0))
        message.angle = self.last_steering
        message.velocity = float(max(0.0, speed))
        self.drive_pub.publish(message)

    def _limited_steering(self, desired_steering, alpha_reduction=0.0, extra_step=0.0):
        desired_steering = clamp(desired_steering, -1.0, 1.0)
        alpha = clamp(
            self.params.steering_smoothing_alpha - max(0.0, alpha_reduction),
            0.0,
            0.98,
        )
        smoothed_steering = (
            alpha * self.last_steering + (1.0 - alpha) * desired_steering
        )
        step = max(0.01, self.params.max_steering_step + max(0.0, extra_step))
        delta = smoothed_steering - self.last_steering
        if delta > step:
            delta = step
        elif delta < -step:
            delta = -step
        return clamp(self.last_steering + delta, -1.0, 1.0)

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
        # Treat non-finite and non-physical near-zero returns as "no obstacle".
        valid_mask = np.isfinite(ranges) & (ranges > max(self.params.min_range, 1e-3))
        ranges[~valid_mask] = self.params.max_range
        ranges = np.clip(ranges, self.params.min_range, self.params.max_range)

        angles = (
            scan.angle_min
            + np.arange(ranges.shape[0], dtype=np.float32) * scan.angle_increment
        )

        half_fov = math.radians(self.params.field_of_view_degrees) * 0.5
        in_fov = np.abs(angles) <= half_fov
        if not np.any(in_fov):
            self._publish_drive(0.0, 0.0)
            return

        fov_ranges = ranges[in_fov].copy()
        fov_ranges_raw = fov_ranges.copy()
        fov_angles = angles[in_fov]

        front_window = math.radians(self.params.front_window_degrees) * 0.5
        front_mask = np.abs(fov_angles) <= front_window
        front_stop_active = False
        if np.any(front_mask):
            front_values = fov_ranges_raw[front_mask]
            percentile = clamp(self.params.front_stop_percentile, 0.0, 100.0)
            front_distance = float(np.percentile(front_values, percentile))
            if front_distance <= self.params.hard_stop_distance:
                front_stop_active = True
                self.front_stop_counter = max(
                    self.front_stop_counter,
                    max(1, self.params.front_stop_hold_frames),
                )
            elif front_distance <= self.params.front_stop_distance:
                self.front_stop_counter += 1
            else:
                self.front_stop_counter = 0

            front_stop_active = front_stop_active or self.front_stop_counter >= max(
                1,
                self.params.front_stop_hold_frames,
            )
        else:
            self.front_stop_counter = 0

        closest_index = int(np.argmin(fov_ranges))
        closest_distance = float(fov_ranges[closest_index])
        adaptive_bubble_degrees = self.params.bubble_angle_degrees + (
            self.params.bubble_turn_gain_degrees * min(1.0, abs(self.last_steering))
        )
        clearance_bubble_degrees = math.degrees(
            math.atan2(
                self.params.side_clearance_min,
                max(closest_distance, self.params.min_range),
            )
        )
        effective_bubble_degrees = max(
            adaptive_bubble_degrees, clearance_bubble_degrees
        )
        if self.params.bubble_max_angle_degrees > 0.0:
            effective_bubble_degrees = min(
                effective_bubble_degrees,
                self.params.bubble_max_angle_degrees,
            )
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
        if gap_start < 0 or gap_end <= gap_start or gap_length < self.params.min_gap_points:
            if front_stop_active or self.params.gap_fallback_speed_scale <= 0.0:
                self._publish_drive(self.last_steering, 0.0)
                return

            obstacle_angle = float(fov_angles[closest_index])
            fallback_desired = (
                0.0
                if abs(obstacle_angle) < 1e-3
                else -math.copysign(1.0, obstacle_angle)
            )
            fallback_steering = self._limited_steering(fallback_desired)
            fallback_speed = self.params.min_speed * min(
                1.0,
                self.params.gap_fallback_speed_scale,
            )
            self._publish_drive(fallback_steering, fallback_speed)
            return

        gap_ranges = fov_ranges[gap_start:gap_end]
        gap_len = gap_ranges.shape[0]
        center_weights = 1.0 - 0.15 * np.abs(
            np.linspace(-1.0, 1.0, gap_len, dtype=np.float32)
        )
        if gap_len >= 9:
            smooth_kernel = np.ones(9, dtype=np.float32) / 9.0
            smoothed = np.convolve(gap_ranges, smooth_kernel, mode="same")
        elif gap_len >= 3:
            smooth_kernel = np.ones(3, dtype=np.float32) / 3.0
            smoothed = np.convolve(gap_ranges, smooth_kernel, mode="same")
        else:
            smoothed = gap_ranges

        scores = smoothed * center_weights
        safe_mask = gap_ranges >= self.params.side_clearance_min
        if np.any(safe_mask):
            scores = np.where(safe_mask, scores, -1.0)
        max_score = float(np.max(scores))
        if max_score <= 0.0:
            target_local = gap_len // 2
        else:
            candidate_mask = scores >= (0.85 * max_score)
            candidate_indices = np.flatnonzero(candidate_mask)
            candidate_weights = np.square(np.maximum(scores[candidate_mask], 1e-3))
            target_local = int(
                round(float(np.average(candidate_indices, weights=candidate_weights)))
            )
        target_index = gap_start + target_local
        target_angle = float(fov_angles[target_index])

        steering_target_angle = math.radians(
            clamp(
                self.params.steering_target_angle_degrees,
                1.0,
                self.params.field_of_view_degrees * 0.5,
            )
        )
        desired_steering = clamp(
            target_angle / max(steering_target_angle, 1e-6),
            -1.0,
            1.0,
        )
        turn_intent = max(
            abs(desired_steering),
            min(1.0, abs(target_angle) / max(half_fov, 1e-6)),
        )

        side_window = math.radians(70.0)
        left_mask = (fov_angles > 0.0) & (fov_angles <= side_window)
        right_mask = (fov_angles < 0.0) & (fov_angles >= -side_window)
        left_distance = (
            float(np.min(fov_ranges_raw[left_mask]))
            if np.any(left_mask)
            else self.params.max_range
        )
        right_distance = (
            float(np.min(fov_ranges_raw[right_mask]))
            if np.any(right_mask)
            else self.params.max_range
        )
        front_preview_window = math.radians(self.params.front_window_degrees) * 0.5
        front_preview_mask = np.abs(fov_angles) <= front_preview_window
        corner_front_distance = (
            float(np.min(fov_ranges_raw[front_preview_mask]))
            if np.any(front_preview_mask)
            else self.params.max_range
        )
        clearance_norm = max(self.params.side_clearance_min, 1e-6)
        side_delta = left_distance - right_distance
        if abs(side_delta) < self.params.wall_bias_deadband_m:
            wall_bias = 0.0
        else:
            wall_bias = self.params.wall_bias_gain * clamp(
                side_delta / clearance_norm, -1.0, 1.0
            )

        desired_steering = clamp(desired_steering + wall_bias, -1.0, 1.0)
        corner_turn_threshold = math.radians(
            max(0.0, self.params.corner_escape_turn_threshold_degrees)
        )
        corner_escape_progress = 0.0
        turning_left = target_angle >= 0.0
        if (
            self.params.corner_escape_bias > 0.0
            and self.params.corner_escape_front_distance > 0.0
            and abs(target_angle) >= corner_turn_threshold
        ):
            turn_side_distance = left_distance if turning_left else right_distance
            outer_side_distance = right_distance if turning_left else left_distance
            inside_wall_gap = outer_side_distance - turn_side_distance
            if (
                corner_front_distance <= self.params.corner_escape_front_distance
                and inside_wall_gap >= self.params.corner_escape_wall_margin
            ):
                front_term = min(
                    1.0,
                    (
                        self.params.corner_escape_front_distance
                        - corner_front_distance
                    )
                    / max(self.params.corner_escape_front_distance, 1e-6),
                )
                wall_term = min(
                    1.0,
                    (inside_wall_gap - self.params.corner_escape_wall_margin)
                    / max(clearance_norm, 1e-6),
                )
                corner_escape_progress = front_term * wall_term
                escape_bias = self.params.corner_escape_bias * corner_escape_progress
                desired_steering += -escape_bias if turning_left else escape_bias
                desired_steering = clamp(desired_steering, -1.0, 1.0)

        turn_intent = max(turn_intent, abs(desired_steering))
        desired_turn_mag = abs(desired_steering)
        sharp_turn_threshold = clamp(
            self.params.sharp_turn_steering_threshold, 0.0, 0.95
        )
        sharp_turn_progress = 0.0
        if desired_turn_mag > sharp_turn_threshold:
            sharp_turn_progress = min(
                1.0,
                (desired_turn_mag - sharp_turn_threshold)
                / max(1e-6, 1.0 - sharp_turn_threshold),
            )

        steering = self._limited_steering(
            desired_steering,
            alpha_reduction=(
                sharp_turn_progress * max(0.0, self.params.sharp_turn_alpha_reduction)
                + corner_escape_progress
                * max(0.0, self.params.corner_escape_alpha_reduction)
            ),
            extra_step=(
                sharp_turn_progress * max(0.0, self.params.sharp_turn_extra_steering_step)
                + corner_escape_progress
                * max(0.0, self.params.corner_escape_extra_steering_step)
            ),
        )

        front_window = math.radians(self.params.front_window_degrees) * 0.5
        front_mask = np.abs(fov_angles) <= front_window
        effective_front_distance = self.params.max_range
        effective_front_min_distance = self.params.max_range
        if np.any(front_mask):
            front_values = fov_ranges_raw[front_mask]
            front_min_distance = float(np.min(front_values))
            percentile = clamp(self.params.front_stop_percentile, 0.0, 100.0)
            front_distance = float(np.percentile(front_values, percentile))
            effective_front_distance = front_distance
            effective_front_min_distance = front_min_distance

            path_window = math.radians(self.params.front_path_window_degrees) * 0.5
            path_mask = np.abs(fov_angles - target_angle) <= path_window
            if np.any(path_mask):
                path_values = fov_ranges_raw[path_mask]
                path_min_distance = float(np.min(path_values))
                path_distance = float(np.percentile(path_values, percentile))
                threshold = math.radians(
                    self.params.front_path_steering_threshold_degrees
                )
                if abs(target_angle) >= threshold:
                    effective_front_distance = path_distance
                    effective_front_min_distance = path_min_distance

            if effective_front_min_distance <= self.params.hard_stop_distance:
                self.front_stop_counter = max(1, self.params.front_stop_hold_frames)
                self._publish_drive(steering, 0.0)
                return

            if effective_front_distance <= self.params.front_stop_distance:
                self.front_stop_counter += 1
            else:
                self.front_stop_counter = 0

            if self.front_stop_counter >= max(1, self.params.front_stop_hold_frames):
                self._publish_drive(steering, 0.0)
                return
        else:
            self.front_stop_counter = 0

        turn_mag = min(1.0, abs(steering))
        turn_intent_weight = clamp(self.params.turn_intent_weight, 0.0, 1.0)
        speed_turn_mag = max(
            turn_mag,
            (1.0 - turn_intent_weight) * turn_mag + turn_intent_weight * turn_intent,
        )
        turn_speed_scale = 1.0 - (
            speed_turn_mag ** self.params.steering_slowdown_exponent
        )
        speed_scale = turn_speed_scale
        min_side_distance = min(left_distance, right_distance)
        side_clearance_scale = min(1.0, min_side_distance / clearance_norm)
        speed_scale *= max(self.params.side_speed_floor, side_clearance_scale)

        caution_distance = max(
            self.params.front_stop_distance + 0.05,
            self.params.front_caution_distance,
        )
        front_progress = (
            effective_front_distance - self.params.front_stop_distance
        ) / (caution_distance - self.params.front_stop_distance)
        front_progress = clamp(front_progress, 0.0, 1.0)
        front_speed_scale = 0.2 + 0.8 * (front_progress**1.5)
        speed_scale *= front_speed_scale

        speed = self.params.min_speed + (
            self.params.max_speed - self.params.min_speed
        ) * speed_scale
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
