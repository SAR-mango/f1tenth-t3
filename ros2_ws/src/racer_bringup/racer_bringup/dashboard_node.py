import math
import sys
import time
from collections import deque
from typing import Deque, List, Optional, Tuple

from PyQt5 import QtCore, QtGui, QtWidgets
from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


ScanPoint = Tuple[float, float]


def project_scan_to_xy(message: LaserScan, max_points: int = 1600) -> Tuple[List[ScanPoint], float]:
    points: List[ScanPoint] = []
    if not message.ranges:
        return (points, _default_extent(message.range_max))

    step = max(1, len(message.ranges) // max_points)
    max_abs_coordinate = 0.0

    for index in range(0, len(message.ranges), step):
        range_value = float(message.ranges[index])
        if not math.isfinite(range_value) or range_value <= 0.0:
            continue
        if message.range_min > 0.0 and range_value < message.range_min:
            continue
        if message.range_max > 0.0 and range_value > message.range_max:
            continue

        angle = message.angle_min + index * message.angle_increment
        x_value = math.cos(angle) * range_value
        y_value = math.sin(angle) * range_value
        points.append((x_value, y_value))
        max_abs_coordinate = max(max_abs_coordinate, abs(x_value), abs(y_value))

    if not points:
        return (points, _default_extent(message.range_max))

    return (points, max(3.0, max_abs_coordinate * 1.1))


def _default_extent(range_max: float) -> float:
    if math.isfinite(range_max) and range_max > 0.0:
        return max(1.0, float(range_max))
    return 5.0


def nice_grid_step(extent_m: float) -> float:
    if extent_m <= 0.0:
        return 1.0

    target_step = extent_m / 4.0
    magnitude = 10.0 ** math.floor(math.log10(target_step))
    for factor in (1.0, 2.0, 5.0, 10.0):
        candidate = factor * magnitude
        if candidate >= target_step:
            return candidate
    return 10.0 * magnitude


class DashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("dashboard_node")

        self.render_period_sec = float(self.declare_parameter("render_period_sec", 0.1).value)
        self.scan_topic = str(self.declare_parameter("scan_topic", "/scan").value)
        self.motion_topic = str(self.declare_parameter("motion_topic", "/cmd_vel").value)
        self.stamped_motion_topic = str(
            self.declare_parameter("stamped_motion_topic", "/telemetry/uart_command").value
        )

        self.scan_arrival_times: Deque[float] = deque()
        self.last_scan_time: Optional[float] = None
        self.latest_scan_points: List[ScanPoint] = []
        self.latest_scan_extent_m = 5.0
        self.last_motion_time: Optional[float] = None
        self.latest_velocity_mps: Optional[float] = None
        self.latest_steering_rad: Optional[float] = None
        self.last_stamped_motion_time: Optional[float] = None
        self.latest_stamped_velocity_mps: Optional[float] = None
        self.latest_stamped_steering_rad: Optional[float] = None

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
            f"motion={self.motion_topic} "
            f"stamped_motion={self.stamped_motion_topic}"
        )

    def scan_callback(self, message: LaserScan) -> None:
        now_sec = time.monotonic()
        self.last_scan_time = now_sec
        self.scan_arrival_times.append(now_sec)
        self._prune_scan_times(now_sec)
        self.latest_scan_points, self.latest_scan_extent_m = project_scan_to_xy(message)

    def motion_callback(self, message: Twist) -> None:
        self.last_motion_time = time.monotonic()
        self.latest_velocity_mps = float(message.linear.x)
        self.latest_steering_rad = float(message.angular.z)

    def stamped_motion_callback(self, message: TwistStamped) -> None:
        self.last_stamped_motion_time = time.monotonic()
        self.latest_stamped_velocity_mps = float(message.twist.linear.x)
        self.latest_stamped_steering_rad = float(message.twist.angular.z)

    def current_motion_values(self) -> Tuple[Optional[float], Optional[float]]:
        use_stamped_motion = self.last_stamped_motion_time is not None and (
            self.last_motion_time is None or self.last_stamped_motion_time >= self.last_motion_time
        )
        if use_stamped_motion:
            return (self.latest_stamped_velocity_mps, self.latest_stamped_steering_rad)
        return (self.latest_velocity_mps, self.latest_steering_rad)

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
        scan_age = now_sec - self.last_scan_time
        return f"{self.scan_rate_hz():4.1f} Hz | last scan {scan_age:4.2f}s ago"

    def _prune_scan_times(self, now_sec: float) -> None:
        cutoff = now_sec - 5.0
        while self.scan_arrival_times and self.scan_arrival_times[0] < cutoff:
            self.scan_arrival_times.popleft()


class MetricCard(QtWidgets.QFrame):
    def __init__(self, title: str, value: str) -> None:
        super().__init__()

        self.title_label = QtWidgets.QLabel(title)
        self.value_label = QtWidgets.QLabel(value)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(18, 14, 18, 14)
        layout.setSpacing(8)
        layout.addWidget(self.title_label)
        layout.addWidget(self.value_label)

        self.title_label.setAlignment(QtCore.Qt.AlignCenter)
        self.value_label.setAlignment(QtCore.Qt.AlignCenter)

        self.title_label.setStyleSheet(
            "color: #94a3b8; font-size: 14px; font-weight: 600; letter-spacing: 0.5px;"
        )
        self.value_label.setStyleSheet(
            "color: #f8fafc; font-size: 34px; font-weight: 700; font-family: 'DejaVu Sans Mono';"
        )
        self.setStyleSheet(
            "QFrame {"
            "background: #0f172a;"
            "border: 1px solid #334155;"
            "border-radius: 12px;"
            "}"
        )

    def set_value(self, value: str) -> None:
        self.value_label.setText(value)


class LidarXYWidget(QtWidgets.QWidget):
    def __init__(self, node: DashboardNode) -> None:
        super().__init__()
        self.node = node
        self.now_sec = time.monotonic()
        self.setMinimumHeight(560)

    def refresh(self, now_sec: float) -> None:
        self.now_sec = now_sec
        self.update()

    def paintEvent(self, _: QtGui.QPaintEvent) -> None:
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        background = QtGui.QColor("#020617")
        panel = QtGui.QColor("#0f172a")
        grid = QtGui.QColor("#1e293b")
        axis = QtGui.QColor("#64748b")
        text = QtGui.QColor("#e2e8f0")
        point_color = QtGui.QColor("#38bdf8")
        origin_color = QtGui.QColor("#f97316")

        painter.fillRect(self.rect(), background)

        panel_rect = QtCore.QRectF(0.0, 0.0, float(self.width()), float(self.height()))
        painter.fillRect(panel_rect, panel)

        plot_bounds = QtCore.QRectF(76.0, 52.0, self.width() - 108.0, self.height() - 110.0)
        if plot_bounds.width() <= 0.0 or plot_bounds.height() <= 0.0:
            return

        plot_size = min(plot_bounds.width(), plot_bounds.height())
        plot_rect = QtCore.QRectF(
            plot_bounds.center().x() - plot_size / 2.0,
            plot_bounds.center().y() - plot_size / 2.0,
            plot_size,
            plot_size,
        )

        extent_m = max(1.0, self.node.latest_scan_extent_m)
        self._draw_grid(painter, plot_rect, extent_m, grid, axis, text)
        self._draw_points(painter, plot_rect, extent_m, point_color)

        origin = plot_rect.center()
        painter.setPen(QtGui.QPen(origin_color, 2.0))
        painter.setBrush(origin_color)
        painter.drawEllipse(origin, 4.0, 4.0)

        painter.setPen(text)
        painter.setFont(QtGui.QFont("DejaVu Sans", 12, QtGui.QFont.Bold))
        painter.drawText(
            QtCore.QRectF(20.0, 12.0, self.width() - 40.0, 20.0),
            QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter,
            "LiDAR XY View",
        )

        painter.setFont(QtGui.QFont("DejaVu Sans", 10))
        painter.drawText(
            QtCore.QRectF(20.0, 30.0, self.width() - 40.0, 18.0),
            QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter,
            f"{self.node.scan_topic} | {self.node.scan_status(self.now_sec)} | {len(self.node.latest_scan_points)} points",
        )

        painter.setPen(QtGui.QColor("#94a3b8"))
        painter.drawText(
            QtCore.QRectF(plot_rect.left(), plot_rect.bottom() + 14.0, plot_rect.width(), 18.0),
            QtCore.Qt.AlignCenter,
            "X (m)",
        )
        painter.drawText(
            QtCore.QRectF(12.0, plot_rect.top() - 6.0, 42.0, 18.0),
            QtCore.Qt.AlignCenter,
            "Y (m)",
        )

        if not self.node.latest_scan_points:
            painter.setPen(QtGui.QColor("#cbd5e1"))
            painter.setFont(QtGui.QFont("DejaVu Sans", 12))
            painter.drawText(
                plot_rect,
                QtCore.Qt.AlignCenter,
                f"Waiting for LiDAR data on {self.node.scan_topic}",
            )

    def _draw_grid(
        self,
        painter: QtGui.QPainter,
        plot_rect: QtCore.QRectF,
        extent_m: float,
        grid_color: QtGui.QColor,
        axis_color: QtGui.QColor,
        text_color: QtGui.QColor,
    ) -> None:
        painter.setPen(QtGui.QPen(grid_color, 1.0))
        grid_step = nice_grid_step(extent_m)
        tick_value = 0.0
        while tick_value <= extent_m + 1e-6:
            for signed_value in ((-tick_value), tick_value):
                point = self._project_point(plot_rect, extent_m, signed_value, 0.0)
                painter.drawLine(
                    QtCore.QPointF(point.x(), plot_rect.top()),
                    QtCore.QPointF(point.x(), plot_rect.bottom()),
                )
                point = self._project_point(plot_rect, extent_m, 0.0, signed_value)
                painter.drawLine(
                    QtCore.QPointF(plot_rect.left(), point.y()),
                    QtCore.QPointF(plot_rect.right(), point.y()),
                )
            tick_value += grid_step

        painter.setPen(QtGui.QPen(axis_color, 1.6))
        center = plot_rect.center()
        painter.drawRect(plot_rect)
        painter.drawLine(
            QtCore.QPointF(center.x(), plot_rect.top()),
            QtCore.QPointF(center.x(), plot_rect.bottom()),
        )
        painter.drawLine(
            QtCore.QPointF(plot_rect.left(), center.y()),
            QtCore.QPointF(plot_rect.right(), center.y()),
        )

        painter.setPen(text_color)
        painter.setFont(QtGui.QFont("DejaVu Sans Mono", 9))
        axis_label_values = (-extent_m, -extent_m / 2.0, 0.0, extent_m / 2.0, extent_m)
        for x_value in axis_label_values:
            point = self._project_point(plot_rect, extent_m, x_value, 0.0)
            painter.drawText(
                QtCore.QRectF(point.x() - 24.0, plot_rect.bottom() + 2.0, 48.0, 14.0),
                QtCore.Qt.AlignCenter,
                f"{x_value:.1f}",
            )
        for y_value in axis_label_values:
            point = self._project_point(plot_rect, extent_m, 0.0, y_value)
            painter.drawText(
                QtCore.QRectF(plot_rect.left() - 58.0, point.y() - 8.0, 48.0, 16.0),
                QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter,
                f"{y_value:.1f}",
            )

    def _draw_points(
        self,
        painter: QtGui.QPainter,
        plot_rect: QtCore.QRectF,
        extent_m: float,
        point_color: QtGui.QColor,
    ) -> None:
        if not self.node.latest_scan_points:
            return

        painter.setPen(QtGui.QPen(point_color, 2.0))
        polygon = QtGui.QPolygonF(
            [
                self._project_point(plot_rect, extent_m, x_value, y_value)
                for x_value, y_value in self.node.latest_scan_points
            ]
        )
        painter.drawPoints(polygon)

    def _project_point(
        self,
        plot_rect: QtCore.QRectF,
        extent_m: float,
        x_value: float,
        y_value: float,
    ) -> QtCore.QPointF:
        half_width = plot_rect.width() / 2.0
        center = plot_rect.center()
        x_pos = center.x() + (x_value / extent_m) * half_width
        y_pos = center.y() - (y_value / extent_m) * half_width
        return QtCore.QPointF(x_pos, y_pos)


class DashboardWindow(QtWidgets.QWidget):
    def __init__(self, node: DashboardNode) -> None:
        super().__init__()
        self.node = node

        self.setWindowTitle("F1TENTH LiDAR Dashboard")
        self.resize(980, 820)
        self.setStyleSheet("background: #020617;")

        outer_layout = QtWidgets.QVBoxLayout(self)
        outer_layout.setContentsMargins(14, 14, 14, 14)
        outer_layout.setSpacing(12)

        header = QtWidgets.QLabel("F1TENTH LiDAR Dashboard")
        header.setStyleSheet(
            "color: #f8fafc; font-size: 22px; font-weight: 700; padding: 4px 2px;"
        )
        outer_layout.addWidget(header)

        self.lidar_plot = LidarXYWidget(self.node)
        outer_layout.addWidget(self.lidar_plot, 1)

        metrics_layout = QtWidgets.QHBoxLayout()
        metrics_layout.setSpacing(12)
        self.velocity_card = MetricCard("Velocity (m/s)", "--")
        self.steering_card = MetricCard("Steering Angle (rad)", "--")
        metrics_layout.addWidget(self.velocity_card)
        metrics_layout.addWidget(self.steering_card)
        outer_layout.addLayout(metrics_layout)

        footer = QtWidgets.QLabel(
            "Numeric readouts use the configured Twist topic when available and stay blank otherwise."
        )
        footer.setStyleSheet("color: #94a3b8; font-size: 12px; padding-left: 2px;")
        outer_layout.addWidget(footer)

    def refresh(self) -> None:
        self.lidar_plot.refresh(time.monotonic())
        velocity_mps, steering_rad = self.node.current_motion_values()
        self.velocity_card.set_value(format_metric(velocity_mps))
        self.steering_card.set_value(format_metric(steering_rad))


def format_metric(value: Optional[float]) -> str:
    if value is None:
        return "--"
    return f"{value: .3f}"


def main(args=None) -> int:
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv if args is None else [sys.argv[0], *args])
    app.setApplicationName("F1TENTH Dashboard")
    app.setStyle("Fusion")

    node = DashboardNode()
    window = DashboardWindow(node)
    window.show()
    window.refresh()

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
    refresh_timer.timeout.connect(window.refresh)
    refresh_timer.start()

    exit_code = 0
    try:
        exit_code = app.exec_()
    finally:
        ros_timer.stop()
        refresh_timer.stop()
        window.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
