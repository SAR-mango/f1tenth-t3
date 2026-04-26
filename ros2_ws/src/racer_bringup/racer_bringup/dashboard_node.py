import math
import sys
import time
from collections import deque
from typing import Deque, Optional, Tuple

import numpy as np
from PyQt5 import QtCore
import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import yaml

try:
    from wallfollowing2 import weighted_pairs_mmse_algorithm
except ImportError as exc:  # pragma: no cover - runtime dependency guard
    weighted_pairs_mmse_algorithm = None
    _WEIGHTED_PAIRS_IMPORT_ERROR = exc
else:
    _WEIGHTED_PAIRS_IMPORT_ERROR = None

try:
    import pyqtgraph as pg
except ImportError as exc:  # pragma: no cover - runtime dependency guard
    pg = None
    _PYQTGRAPH_IMPORT_ERROR = exc
else:
    _PYQTGRAPH_IMPORT_ERROR = None


DEFAULT_LIDAR_PLOT_LATERAL_LIMIT_M = 4.0
DEFAULT_LIDAR_PLOT_FORWARD_LIMIT_M = 8.0
DEFAULT_COMMAND_HISTORY_WINDOW_S = 10.0
DEFAULT_CAR_WIDTH_M = 0.2
DEFAULT_EMERGENCY_STOP_DISTANCE_M = 0.35
DEFAULT_EMERGENCY_STOP_HALF_ANGLE_DEG = 20.0
DEFAULT_HEADING_MARKER_LENGTH_M = 0.35
DEFAULT_PLOT_COLOR_HEX = "#ff5a5f"
STAMPED_MOTION_STALE_TIMEOUT_SEC = 0.75
OVERLAY_MODE_NONE = "none"
OVERLAY_MODE_WEIGHTED_PAIRS_MMSE = "weighted_pairs_mmse"
WEIGHTED_PAIRS_PARAM_FILE_NODE_KEY = "weighted_pairs_mmse"
PAIR_LINE_WIDTH_MIN = 2.4
PAIR_LINE_WIDTH_MAX = 4.2
PAIR_LINE_GREEN_MIN = 96
PAIR_LINE_GREEN_MAX = 255
PAIR_LINE_ALPHA_MIN = 120
PAIR_LINE_ALPHA_MAX = 255


def project_scan_to_local(
    message: LaserScan,
    max_points: int = 1600,
) -> Tuple[np.ndarray, np.ndarray]:
    if len(message.ranges) == 0:
        return (
            np.array([], dtype=np.float64),
            np.array([], dtype=np.float64),
        )

    step = max(1, len(message.ranges) // max_points)
    lateral_points = []
    forward_points = []

    for index in range(0, len(message.ranges), step):
        range_value = float(message.ranges[index])
        if not math.isfinite(range_value) or range_value <= 0.0:
            continue
        if message.range_min > 0.0 and range_value < message.range_min:
            continue
        if message.range_max > 0.0 and range_value > message.range_max:
            continue

        angle = message.angle_min + index * message.angle_increment
        forward_points.append(math.cos(angle) * range_value)
        lateral_points.append(math.sin(angle) * range_value)

    return (
        np.asarray(lateral_points, dtype=np.float64),
        np.asarray(forward_points, dtype=np.float64),
    )


def segments_to_connected_data(segments_xy: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    if segments_xy is None or len(segments_xy) == 0:
        return (
            np.array([], dtype=np.float64),
            np.array([], dtype=np.float64),
        )

    segments_xy = np.asarray(segments_xy, dtype=np.float64)
    point_count = len(segments_xy)
    x_values = np.empty(3 * point_count, dtype=np.float64)
    y_values = np.empty(3 * point_count, dtype=np.float64)
    x_values[0::3] = segments_xy[:, 0, 0]
    y_values[0::3] = segments_xy[:, 0, 1]
    x_values[1::3] = segments_xy[:, 1, 0]
    y_values[1::3] = segments_xy[:, 1, 1]
    x_values[2::3] = np.nan
    y_values[2::3] = np.nan
    return x_values, y_values


def scan_to_algorithm_input(message: LaserScan) -> dict:
    if len(message.ranges) == 0:
        return {
            "angles_deg": np.zeros(0, dtype=np.float64),
            "ranges_m": np.zeros(0, dtype=np.float64),
            "hit_valid": np.zeros(0, dtype=bool),
        }

    ranges_m = np.asarray(message.ranges, dtype=np.float64)
    angles_rad = message.angle_min + np.arange(
        ranges_m.shape[0],
        dtype=np.float64,
    ) * message.angle_increment

    hit_valid = np.isfinite(ranges_m) & (ranges_m > 0.0)
    if message.range_min > 0.0:
        hit_valid &= ranges_m >= message.range_min
    if math.isfinite(message.range_max) and message.range_max > 0.0:
        hit_valid &= ranges_m <= message.range_max

    return {
        "angles_deg": np.rad2deg(angles_rad),
        "ranges_m": ranges_m,
        "hit_valid": hit_valid,
    }


def load_ros_parameters_from_yaml(path: str, node_key: str) -> dict:
    with open(path, "r", encoding="utf-8") as stream:
        document = yaml.safe_load(stream) or {}

    if not isinstance(document, dict):
        raise ValueError(f"{path} does not contain a ROS parameter mapping")

    candidate_keys = [node_key, "/**", "dashboard_node"]
    for candidate_key in candidate_keys:
        section = document.get(candidate_key)
        if not isinstance(section, dict):
            continue
        ros_parameters = section.get("ros__parameters")
        if isinstance(ros_parameters, dict):
            return dict(ros_parameters)

    for section in document.values():
        if not isinstance(section, dict):
            continue
        ros_parameters = section.get("ros__parameters")
        if isinstance(ros_parameters, dict):
            return dict(ros_parameters)

    raise ValueError(f"{path} does not define any ros__parameters section")


def build_weighted_pairs_pair_segments(scan_input: dict, overlay_debug: dict) -> Tuple[list, np.ndarray]:
    if weighted_pairs_mmse_algorithm is None:
        return ([], np.array([], dtype=np.float64))

    preprocessed_scan = weighted_pairs_mmse_algorithm.preprocess_lidar(scan_input)
    if preprocessed_scan is None:
        return ([], np.array([], dtype=np.float64))

    valid_cones = weighted_pairs_mmse_algorithm.extract_valid_cones(preprocessed_scan)
    pair_angles_deg = np.asarray(overlay_debug.get("pair_angles_deg", []), dtype=np.float64)
    pair_values = np.asarray(overlay_debug.get("pair_values", []), dtype=np.float64)

    pair_segments = []
    pair_weights = []

    for pair_angle_deg, pair_value in zip(pair_angles_deg, pair_values):
        left_cone = valid_cones.get(int(round(float(pair_angle_deg))))
        right_cone = valid_cones.get(int(round(-float(pair_angle_deg))))
        if left_cone is None or right_cone is None:
            continue

        pair_segments.append(
            np.asarray(
                [
                    [np.zeros(2, dtype=np.float64), left_cone["centerline_endpoint_local"]],
                    [np.zeros(2, dtype=np.float64), right_cone["centerline_endpoint_local"]],
                ],
                dtype=np.float64,
            )
        )
        pair_weights.append(float(pair_value))

    return (pair_segments, np.asarray(pair_weights, dtype=np.float64))


def emergency_stop_guide_segments(
    distance_m: float,
    half_angle_deg: float,
) -> np.ndarray:
    angle_rad = math.radians(half_angle_deg)
    return np.asarray(
        [
            [[0.0, 0.0], [distance_m * math.sin(angle_rad), distance_m * math.cos(angle_rad)]],
            [[0.0, 0.0], [-distance_m * math.sin(angle_rad), distance_m * math.cos(angle_rad)]],
        ],
        dtype=np.float64,
    )


class DashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("dashboard_node")

        self.render_period_sec = float(self.declare_parameter("render_period_sec", 0.1).value)
        self.history_window_sec = float(
            self.declare_parameter("history_window_sec", DEFAULT_COMMAND_HISTORY_WINDOW_S).value
        )
        self.scan_topic = str(self.declare_parameter("scan_topic", "/scan").value)
        self.motion_topic = str(self.declare_parameter("motion_topic", "/cmd_vel").value)
        self.stamped_motion_topic = str(
            self.declare_parameter("stamped_motion_topic", "/telemetry/uart_command").value
        )
        self.lidar_plot_lateral_limit_m = float(
            self.declare_parameter(
                "lidar_plot_lateral_limit_m",
                DEFAULT_LIDAR_PLOT_LATERAL_LIMIT_M,
            ).value
        )
        self.lidar_plot_forward_limit_m = float(
            self.declare_parameter(
                "lidar_plot_forward_limit_m",
                DEFAULT_LIDAR_PLOT_FORWARD_LIMIT_M,
            ).value
        )
        self.car_width_m = float(
            self.declare_parameter("car_width_m", DEFAULT_CAR_WIDTH_M).value
        )
        self.emergency_stop_distance_m = float(
            self.declare_parameter(
                "emergency_stop_distance_m",
                DEFAULT_EMERGENCY_STOP_DISTANCE_M,
            ).value
        )
        self.emergency_stop_half_angle_deg = float(
            self.declare_parameter(
                "emergency_stop_half_angle_deg",
                DEFAULT_EMERGENCY_STOP_HALF_ANGLE_DEG,
            ).value
        )
        self.steering_plot_title = str(
            self.declare_parameter("steering_plot_title", "Steering Command").value
        )
        self.steering_axis_label = str(
            self.declare_parameter("steering_axis_label", "Command").value
        )
        self.steering_zero_is_straight = bool(
            self.declare_parameter("steering_zero_is_straight", False).value
        )
        self.overlay_mode = str(
            self.declare_parameter("overlay_mode", OVERLAY_MODE_NONE).value
        ).strip().lower()
        self.weighted_pairs_params_file = str(
            self.declare_parameter("weighted_pairs_params_file", "").value
        ).strip()

        self.scan_arrival_times: Deque[float] = deque()
        self.last_scan_time: Optional[float] = None
        self.latest_scan_message: Optional[LaserScan] = None
        self.latest_scan_lateral_m = np.array([], dtype=np.float64)
        self.latest_scan_forward_m = np.array([], dtype=np.float64)

        self.last_motion_time: Optional[float] = None
        self.latest_velocity_mps: Optional[float] = None
        self.latest_steering_value: Optional[float] = None

        self.last_stamped_motion_time: Optional[float] = None
        self.latest_stamped_velocity_mps: Optional[float] = None
        self.latest_stamped_steering_value: Optional[float] = None

        self.motion_history_times: Deque[float] = deque()
        self.motion_history_speed_mps: Deque[float] = deque()
        self.motion_history_steering: Deque[float] = deque()

        self.overlay_emergency_stop_distance_m = self.emergency_stop_distance_m
        self.overlay_emergency_stop_half_angle_deg = self.emergency_stop_half_angle_deg
        self._last_overlay_error_message = ""
        self._last_overlay_error_log_time = 0.0
        self._configure_overlay()

        self._scan_subscription = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            10,
        )
        self._motion_subscription = None
        if self.motion_topic:
            self._motion_subscription = self.create_subscription(
                Twist,
                self.motion_topic,
                self.motion_callback,
                10,
            )
        self._stamped_motion_subscription = None
        if self.stamped_motion_topic:
            self._stamped_motion_subscription = self.create_subscription(
                TwistStamped,
                self.stamped_motion_topic,
                self.stamped_motion_callback,
                10,
            )

        self.get_logger().info(
            "Dashboard ready. "
            f"scan={self.scan_topic} "
            f"motion={self.motion_topic or '<disabled>'} "
            f"stamped_motion={self.stamped_motion_topic or '<disabled>'} "
            f"steering_plot_title={self.steering_plot_title} "
            f"steering_axis_label={self.steering_axis_label} "
            f"overlay_mode={self.overlay_mode}"
        )

    def scan_callback(self, message: LaserScan) -> None:
        now_sec = time.monotonic()
        self.last_scan_time = now_sec
        self.scan_arrival_times.append(now_sec)
        self._prune_scan_times(now_sec)
        self.latest_scan_message = message
        self.latest_scan_lateral_m, self.latest_scan_forward_m = project_scan_to_local(message)

    def motion_callback(self, message: Twist) -> None:
        now_sec = time.monotonic()
        speed_mps = float(message.linear.x)
        steering_value = float(message.angular.z)

        self.last_motion_time = now_sec
        self.latest_velocity_mps = speed_mps
        self.latest_steering_value = steering_value

        if not self._stamped_source_is_active(now_sec):
            self._append_motion_sample(now_sec, speed_mps, steering_value)

    def stamped_motion_callback(self, message: TwistStamped) -> None:
        now_sec = time.monotonic()
        speed_mps = float(message.twist.linear.x)
        steering_value = float(message.twist.angular.z)

        self.last_stamped_motion_time = now_sec
        self.latest_stamped_velocity_mps = speed_mps
        self.latest_stamped_steering_value = steering_value
        self._append_motion_sample(now_sec, speed_mps, steering_value)

    def current_motion_values(self) -> Tuple[Optional[float], Optional[float]]:
        now_sec = time.monotonic()
        if self._stamped_source_is_active(now_sec):
            return (
                self.latest_stamped_velocity_mps,
                self.latest_stamped_steering_value,
            )
        return (self.latest_velocity_mps, self.latest_steering_value)

    def scan_rate_hz(self) -> float:
        if len(self.scan_arrival_times) < 2:
            return 0.0
        duration = self.scan_arrival_times[-1] - self.scan_arrival_times[0]
        if duration <= 0.0:
            return 0.0
        return (len(self.scan_arrival_times) - 1) / duration

    def scan_status(self, now_sec: float) -> str:
        if self.last_scan_time is None:
            return f"waiting for scan on {self.scan_topic}"
        scan_age_sec = now_sec - self.last_scan_time
        return f"{self.scan_rate_hz():4.1f} Hz | last scan {scan_age_sec:4.2f}s ago"

    def history_arrays(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        if not self.motion_history_times:
            empty = np.array([], dtype=np.float64)
            return (empty, empty, empty)

        times = np.asarray(self.motion_history_times, dtype=np.float64)
        latest_time = times[-1]
        relative_time = times - latest_time
        speed_values = np.asarray(self.motion_history_speed_mps, dtype=np.float64)
        steering_values = np.asarray(self.motion_history_steering, dtype=np.float64).copy()

        if self.steering_zero_is_straight:
            steering_values[np.isclose(steering_values, 0.0, atol=1e-9)] = np.nan
        steering_values[~np.isfinite(steering_values)] = np.nan

        return (relative_time, speed_values, steering_values)

    def _append_motion_sample(self, now_sec: float, speed_mps: float, steering_value: float) -> None:
        self.motion_history_times.append(now_sec)
        self.motion_history_speed_mps.append(speed_mps)
        self.motion_history_steering.append(steering_value)
        self._prune_motion_history(now_sec)

    def _prune_scan_times(self, now_sec: float) -> None:
        cutoff = now_sec - 5.0
        while self.scan_arrival_times and self.scan_arrival_times[0] < cutoff:
            self.scan_arrival_times.popleft()

    def _prune_motion_history(self, now_sec: float) -> None:
        cutoff = now_sec - max(1.0, self.history_window_sec)
        while self.motion_history_times and self.motion_history_times[0] < cutoff:
            self.motion_history_times.popleft()
            self.motion_history_speed_mps.popleft()
            self.motion_history_steering.popleft()

    def _stamped_source_is_active(self, now_sec: float) -> bool:
        if not self.stamped_motion_topic or self.last_stamped_motion_time is None:
            return False
        return (now_sec - self.last_stamped_motion_time) <= STAMPED_MOTION_STALE_TIMEOUT_SEC

    def _configure_overlay(self) -> None:
        if self.overlay_mode == OVERLAY_MODE_NONE:
            return

        if self.overlay_mode != OVERLAY_MODE_WEIGHTED_PAIRS_MMSE:
            self.get_logger().warn(
                f"Unknown overlay_mode={self.overlay_mode!r}; disabling overlay rendering."
            )
            self.overlay_mode = OVERLAY_MODE_NONE
            return

        if weighted_pairs_mmse_algorithm is None:
            self.get_logger().warn(
                "weighted_pairs_mmse overlay requested but wallfollowing2 is unavailable; "
                f"disabling overlay rendering. {_WEIGHTED_PAIRS_IMPORT_ERROR}"
            )
            self.overlay_mode = OVERLAY_MODE_NONE
            return

        ros_parameters = {}
        if self.weighted_pairs_params_file:
            try:
                ros_parameters = load_ros_parameters_from_yaml(
                    self.weighted_pairs_params_file,
                    WEIGHTED_PAIRS_PARAM_FILE_NODE_KEY,
                )
            except Exception as exc:
                self.get_logger().warn(
                    "Failed to load weighted-pairs dashboard overlay parameters from "
                    f"{self.weighted_pairs_params_file}: {exc}. Using algorithm defaults."
                )

        weighted_pairs_mmse_algorithm.configure_parameters(
            {
                name: ros_parameters[name]
                for name in weighted_pairs_mmse_algorithm.PARAM_DEFAULTS.keys()
                if name in ros_parameters
            }
        )

        if "front_stop_distance_m" in ros_parameters:
            self.overlay_emergency_stop_distance_m = float(ros_parameters["front_stop_distance_m"])
        if "front_stop_half_angle_deg" in ros_parameters:
            self.overlay_emergency_stop_half_angle_deg = float(
                ros_parameters["front_stop_half_angle_deg"]
            )

    def current_weighted_pairs_overlay(self) -> Optional[dict]:
        if (
            self.overlay_mode != OVERLAY_MODE_WEIGHTED_PAIRS_MMSE
            or weighted_pairs_mmse_algorithm is None
            or self.latest_scan_message is None
        ):
            return None

        try:
            scan_input = scan_to_algorithm_input(self.latest_scan_message)
            overlay_result = weighted_pairs_mmse_algorithm.runAutonomousAlgorithm(scan_input)
            overlay_debug = overlay_result.setdefault("debug", {})
            pair_segments_local, pair_segment_weights = build_weighted_pairs_pair_segments(
                scan_input,
                overlay_debug,
            )
            overlay_debug["pair_segments_local"] = pair_segments_local
            overlay_debug["pair_segment_weights"] = pair_segment_weights
            return overlay_result
        except Exception as exc:
            self._maybe_log_overlay_error(
                f"weighted_pairs dashboard overlay failed; clearing overlay: {exc}"
            )
            return None

    def _maybe_log_overlay_error(self, message: str) -> None:
        now_sec = time.monotonic()
        if (
            message != self._last_overlay_error_message
            or now_sec - self._last_overlay_error_log_time >= 1.0
        ):
            self.get_logger().warn(message)
            self._last_overlay_error_message = message
            self._last_overlay_error_log_time = now_sec


class DashboardUi:
    def __init__(self, node: DashboardNode) -> None:
        if pg is None:  # pragma: no cover - handled before UI is created
            raise RuntimeError("pyqtgraph is required to create the dashboard UI")

        self.node = node
        self.plot_color_hex = DEFAULT_PLOT_COLOR_HEX
        self.pair_line_items = []

        self.lidar_widget, self.lidar_plot, self.lidar_items = self._create_lidar_window()
        self.command_widget, self.command_items = self._create_command_window()

    def show(self) -> None:
        self.lidar_widget.show()
        self.command_widget.show()

    def close(self) -> None:
        self.command_widget.close()
        self.lidar_widget.close()

    def refresh(self) -> None:
        self._refresh_lidar_window()
        self._refresh_command_window()

    def _create_lidar_window(self):
        widget = pg.PlotWidget(title="LiDAR View")
        widget.setWindowTitle("LiDAR View")
        widget.resize(1000, 600)

        plot = widget.getPlotItem()
        plot.setLabel("bottom", "Lateral Position (m)")
        plot.setLabel("left", "Forward Position (m)")
        plot.setAspectLocked(True)
        plot.getViewBox().setMouseEnabled(x=False, y=False)
        plot.getViewBox().invertX(True)
        plot.setXRange(
            -self.node.lidar_plot_lateral_limit_m,
            self.node.lidar_plot_lateral_limit_m,
            padding=0.0,
        )
        plot.setYRange(
            0.0,
            self.node.lidar_plot_forward_limit_m,
            padding=0.0,
        )

        lidar_points_item = pg.ScatterPlotItem(
            size=6,
            brush=pg.mkBrush(self.plot_color_hex),
            pen=None,
        )
        emergency_stop_item = pg.PlotDataItem(
            pen=pg.mkPen("#ff3030", width=1.8),
            connect="finite",
        )
        car_width_item = pg.PlotDataItem(
            pen=pg.mkPen(self.plot_color_hex, width=2.0),
            connect="finite",
        )
        left_boundary_item = pg.PlotDataItem(
            pen=pg.mkPen(self.plot_color_hex, width=1.8),
        )
        right_boundary_item = pg.PlotDataItem(
            pen=pg.mkPen(self.plot_color_hex, width=1.8),
        )
        centerline_item = pg.PlotDataItem(
            pen=pg.mkPen(self.plot_color_hex, width=2.2),
        )
        lookahead_item = pg.ScatterPlotItem(
            size=9,
            brush=pg.mkBrush(self.plot_color_hex),
            pen=None,
        )
        lidar_origin_item = pg.ScatterPlotItem(
            size=7,
            brush=pg.mkBrush(self.plot_color_hex),
            pen=None,
        )
        heading_line_item = pg.PlotDataItem(
            pen=pg.mkPen(self.plot_color_hex, width=2.0),
        )
        heading_head_item = pg.ArrowItem(
            angle=90.0,
            headLen=12.0,
            tipAngle=50.0,
            baseAngle=20.0,
            tailLen=0.0,
            brush=pg.mkBrush(self.plot_color_hex),
            pen=pg.mkPen(self.plot_color_hex),
        )

        for item in [
            emergency_stop_item,
            car_width_item,
            lidar_points_item,
            left_boundary_item,
            right_boundary_item,
            centerline_item,
            lookahead_item,
            lidar_origin_item,
            heading_line_item,
            heading_head_item,
        ]:
            plot.addItem(item)

        guide_segments = emergency_stop_guide_segments(
            self.node.overlay_emergency_stop_distance_m,
            self.node.overlay_emergency_stop_half_angle_deg,
        )
        guide_x, guide_y = segments_to_connected_data(guide_segments)
        emergency_stop_item.setData(guide_x, guide_y)

        half_car_width_m = 0.5 * self.node.car_width_m
        car_width_segment = np.asarray(
            [[[-half_car_width_m, 0.0], [half_car_width_m, 0.0]]],
            dtype=np.float64,
        )
        car_width_x, car_width_y = segments_to_connected_data(car_width_segment)
        car_width_item.setData(car_width_x, car_width_y)
        lidar_origin_item.setData([0.0], [0.0])
        heading_line_item.setData(
            [0.0, 0.0],
            [0.0, DEFAULT_HEADING_MARKER_LENGTH_M],
        )
        heading_head_item.setPos(0.0, DEFAULT_HEADING_MARKER_LENGTH_M)

        return widget, plot, {
            "lidar_points": lidar_points_item,
            "left_boundary": left_boundary_item,
            "right_boundary": right_boundary_item,
            "centerline": centerline_item,
            "lookahead": lookahead_item,
        }

    def _create_command_window(self):
        widget = pg.GraphicsLayoutWidget(title="Command History")
        widget.setWindowTitle("Command History")
        widget.resize(900, 500)

        speed_plot = widget.addPlot(row=0, col=0, title="Speed Command")
        speed_plot.setLabel("left", "Speed (m/s)")
        speed_plot.setLabel("bottom", "Time (s)")
        speed_plot.showGrid(x=True, y=True, alpha=0.25)
        speed_plot.setXRange(-self.node.history_window_sec, 0.0, padding=0.0)

        steering_plot = widget.addPlot(row=1, col=0, title=self.node.steering_plot_title)
        steering_plot.setLabel("left", self.node.steering_axis_label)
        steering_plot.setLabel("bottom", "Time (s)")
        steering_plot.showGrid(x=True, y=True, alpha=0.25)
        steering_plot.setXRange(-self.node.history_window_sec, 0.0, padding=0.0)

        speed_curve = speed_plot.plot(
            pen=pg.mkPen(self.plot_color_hex, width=2.5),
        )
        steering_curve = steering_plot.plot(
            pen=pg.mkPen(self.plot_color_hex, width=2.5),
        )
        speed_mean_line = pg.InfiniteLine(
            angle=0,
            movable=False,
            pen=pg.mkPen(
                self.plot_color_hex,
                width=1.5,
                style=QtCore.Qt.PenStyle.DashLine,
            ),
        )
        speed_plot.addItem(speed_mean_line)
        speed_mean_line.hide()

        return widget, {
            "speed_curve": speed_curve,
            "steering_curve": steering_curve,
            "speed_mean_line": speed_mean_line,
        }

    def _refresh_lidar_window(self) -> None:
        self.lidar_items["lidar_points"].setData(
            self.node.latest_scan_lateral_m,
            self.node.latest_scan_forward_m,
        )
        self._refresh_overlay_items()
        now_sec = time.monotonic()
        self.lidar_plot.setTitle(
            "LiDAR View"
            f"  |  {self.node.scan_topic}"
            f"  |  {self.node.scan_status(now_sec)}"
            f"  |  {len(self.node.latest_scan_lateral_m)} points"
        )

    def _refresh_overlay_items(self) -> None:
        overlay_result = self.node.current_weighted_pairs_overlay()
        if overlay_result is None:
            self._clear_overlay_items()
            return

        debug = overlay_result.get("debug", {})

        self._refresh_pair_line_items(
            debug.get("pair_segments_local", []),
            debug.get("pair_segment_weights", np.array([], dtype=np.float64)),
        )

        if debug.get("left_boundary_local") is not None:
            self.lidar_items["left_boundary"].setData(
                debug["left_boundary_local"][:, 1],
                debug["left_boundary_local"][:, 0],
            )
        else:
            self.lidar_items["left_boundary"].setData([], [])

        if debug.get("right_boundary_local") is not None:
            self.lidar_items["right_boundary"].setData(
                debug["right_boundary_local"][:, 1],
                debug["right_boundary_local"][:, 0],
            )
        else:
            self.lidar_items["right_boundary"].setData([], [])

        if debug.get("path_local") is not None:
            self.lidar_items["centerline"].setData(
                debug["path_local"][:, 1],
                debug["path_local"][:, 0],
            )
        else:
            self.lidar_items["centerline"].setData([], [])

        if debug.get("target_point_local") is not None:
            self.lidar_items["lookahead"].setData(
                [debug["target_point_local"][1]],
                [debug["target_point_local"][0]],
            )
        else:
            self.lidar_items["lookahead"].setData([], [])

    def _clear_overlay_items(self) -> None:
        self._ensure_pair_line_items(0)
        self.lidar_items["left_boundary"].setData([], [])
        self.lidar_items["right_boundary"].setData([], [])
        self.lidar_items["centerline"].setData([], [])
        self.lidar_items["lookahead"].setData([], [])

    def _ensure_pair_line_items(self, pair_count: int) -> None:
        while len(self.pair_line_items) < pair_count:
            pair_item = pg.PlotDataItem(connect="finite")
            self.lidar_plot.addItem(pair_item)
            self.pair_line_items.append(pair_item)

        while len(self.pair_line_items) > pair_count:
            pair_item = self.pair_line_items.pop()
            self.lidar_plot.removeItem(pair_item)

    def _refresh_pair_line_items(self, pair_segments_local, pair_weights) -> None:
        if pair_segments_local is None:
            pair_segments_local = []

        pair_weights = np.asarray(pair_weights, dtype=np.float64)
        pair_count = len(pair_segments_local)
        if pair_count == 0:
            self._ensure_pair_line_items(0)
            return

        self._ensure_pair_line_items(pair_count)

        finite_weights = pair_weights[np.isfinite(pair_weights)]
        if finite_weights.size == 0:
            brightness_values = np.zeros(pair_count, dtype=np.float64)
        else:
            min_weight = float(np.min(finite_weights))
            max_weight = float(np.max(finite_weights))
            if max_weight - min_weight <= 1.0e-9:
                brightness_values = np.ones(pair_count, dtype=np.float64)
            else:
                brightness_values = (pair_weights - min_weight) / (max_weight - min_weight)
                brightness_values[~np.isfinite(brightness_values)] = 0.0
                brightness_values = np.clip(brightness_values, 0.0, 1.0)

        brightness_values = np.power(brightness_values, 0.75)

        for pair_item, pair_segments, brightness in zip(
            self.pair_line_items,
            pair_segments_local,
            brightness_values,
        ):
            line_green = int(
                round(
                    PAIR_LINE_GREEN_MIN
                    + (PAIR_LINE_GREEN_MAX - PAIR_LINE_GREEN_MIN) * float(brightness)
                )
            )
            line_alpha = int(
                round(
                    PAIR_LINE_ALPHA_MIN
                    + (PAIR_LINE_ALPHA_MAX - PAIR_LINE_ALPHA_MIN) * float(brightness)
                )
            )
            line_width = (
                PAIR_LINE_WIDTH_MIN
                + (PAIR_LINE_WIDTH_MAX - PAIR_LINE_WIDTH_MIN) * float(brightness)
            )
            pair_pen = pg.mkPen(
                pg.mkColor((0, line_green, 0, line_alpha)),
                width=line_width,
            )
            pair_x, pair_y = segments_to_connected_data(pair_segments)
            pair_item.setPen(pair_pen)
            pair_item.setData(pair_y, pair_x)

    def _refresh_command_window(self) -> None:
        relative_time, speed_values, steering_values = self.node.history_arrays()

        if len(relative_time) == 0:
            self.command_items["speed_curve"].setData([], [])
            self.command_items["steering_curve"].setData([], [])
            self.command_items["speed_mean_line"].hide()
            return

        self.command_items["speed_curve"].setData(relative_time, speed_values)
        self.command_items["steering_curve"].setData(relative_time, steering_values)
        self.command_items["speed_mean_line"].setValue(float(np.mean(speed_values)))
        self.command_items["speed_mean_line"].show()


def main(args=None) -> int:
    if pg is None:
        sys.stderr.write(
            "dashboard_node requires pyqtgraph. Install `python3-pyqtgraph` and relaunch.\n"
        )
        if _PYQTGRAPH_IMPORT_ERROR is not None:
            sys.stderr.write(f"{_PYQTGRAPH_IMPORT_ERROR}\n")
        return 1

    rclpy.init(args=args)
    pg.setConfigOptions(antialias=False)
    app = pg.mkQApp("F1TENTH Dashboard")

    node = DashboardNode()
    ui = DashboardUi(node)
    ui.show()
    ui.refresh()

    ros_timer = QtCore.QTimer()
    ros_timer.setInterval(20)

    def pump_ros() -> None:
        if not rclpy.ok():
            app.quit()
            return
        rclpy.spin_once(node, timeout_sec=0.0)

    ros_timer.timeout.connect(pump_ros)
    ros_timer.start()

    refresh_timer = QtCore.QTimer()
    refresh_timer.setInterval(max(20, int(node.render_period_sec * 1000.0)))
    refresh_timer.timeout.connect(ui.refresh)
    refresh_timer.start()

    exit_code = 0
    try:
        exit_code = app.exec_()
    finally:
        ros_timer.stop()
        refresh_timer.stop()
        ui.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
