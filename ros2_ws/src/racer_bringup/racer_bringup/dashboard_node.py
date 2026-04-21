import sys
import time
from collections import deque
from typing import Deque, List, Optional, Sequence, Tuple

from PyQt5 import QtCore, QtGui, QtWidgets
from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float64, Int32, String


class TimeSeries:
    def __init__(self) -> None:
        self.samples: Deque[Tuple[float, float]] = deque()

    def append(self, timestamp_sec: float, value: float) -> None:
        self.samples.append((timestamp_sec, value))

    def prune(self, now_sec: float, window_sec: float) -> None:
        cutoff = now_sec - window_sec
        while self.samples and self.samples[0][0] < cutoff:
            self.samples.popleft()

    def points(self, now_sec: float, window_sec: float) -> Tuple[List[float], List[float]]:
        self.prune(now_sec, window_sec)
        return (
            [timestamp - now_sec for timestamp, _ in self.samples],
            [value for _, value in self.samples],
        )

    def latest_value(self) -> Optional[float]:
        if not self.samples:
            return None
        return self.samples[-1][1]


def compute_limits(
    value_sets: Sequence[Sequence[float]],
    default_limits: Tuple[float, float],
) -> Tuple[float, float]:
    values = [value for value_set in value_sets for value in value_set]
    if not values:
        return default_limits

    minimum = min(values)
    maximum = max(values)
    if abs(maximum - minimum) < 1e-6:
        padding = max(0.1, abs(maximum) * 0.2 + 0.05)
    else:
        padding = (maximum - minimum) * 0.2

    lower = minimum - padding
    upper = maximum + padding
    if lower == upper:
        return default_limits
    return (lower, upper)


class DashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("dashboard_node")

        self.window_sec = float(self.declare_parameter("window_sec", 60.0).value)
        self.render_period_sec = float(self.declare_parameter("render_period_sec", 0.1).value)
        self.scan_topic = str(self.declare_parameter("scan_topic", "/scan").value)
        self.uart_telemetry_topic = str(
            self.declare_parameter("uart_telemetry_topic", "/telemetry/uart_command").value
        )
        self.uart_frame_topic = str(
            self.declare_parameter("uart_frame_topic", "/telemetry/uart_frame").value
        )
        self.uart_stale_topic = str(
            self.declare_parameter("uart_stale_topic", "/telemetry/uart_command_stale").value
        )
        self.measured_speed_topic = str(
            self.declare_parameter("measured_speed_topic", "/telemetry/measured_speed").value
        )
        self.measured_steering_topic = str(
            self.declare_parameter("measured_steering_topic", "/telemetry/measured_steering").value
        )
        self.drive_mode_topic = str(
            self.declare_parameter("drive_mode_topic", "/commands/drive_mode").value
        )
        self.emergency_stop_topic = str(
            self.declare_parameter("emergency_stop_topic", "/commands/emergency_stop").value
        )

        self.command_speed = TimeSeries()
        self.command_steering = TimeSeries()
        self.command_brake = TimeSeries()
        self.measured_speed = TimeSeries()
        self.measured_steering = TimeSeries()
        self.scan_arrival_times: Deque[float] = deque()

        self.last_scan_time: Optional[float] = None
        self.latest_uart_frame = "waiting for UART bridge telemetry"
        self.latest_uart_stale = False
        self.latest_drive_mode: Optional[int] = None
        self.latest_emergency_stop = False

        self._subscription_handles = [
            self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10),
            self.create_subscription(
                TwistStamped,
                self.uart_telemetry_topic,
                self.uart_callback,
                10,
            ),
            self.create_subscription(String, self.uart_frame_topic, self.uart_frame_callback, 10),
            self.create_subscription(Bool, self.uart_stale_topic, self.uart_stale_callback, 10),
            self.create_subscription(
                Float64,
                self.measured_speed_topic,
                self.measured_speed_callback,
                10,
            ),
            self.create_subscription(
                Float64,
                self.measured_steering_topic,
                self.measured_steering_callback,
                10,
            ),
            self.create_subscription(Int32, self.drive_mode_topic, self.drive_mode_callback, 10),
            self.create_subscription(
                Bool,
                self.emergency_stop_topic,
                self.emergency_stop_callback,
                10,
            ),
        ]

        self.get_logger().info(
            "Dashboard ready. "
            f"scan={self.scan_topic} "
            f"uart={self.uart_telemetry_topic} "
            f"measured_speed={self.measured_speed_topic} "
            f"measured_steering={self.measured_steering_topic} "
            f"window={self.window_sec:.1f}s"
        )

    def scan_callback(self, _: LaserScan) -> None:
        now_sec = time.monotonic()
        self.last_scan_time = now_sec
        self.scan_arrival_times.append(now_sec)
        self._prune_scan_times(now_sec)

    def uart_callback(self, message: TwistStamped) -> None:
        now_sec = time.monotonic()
        self.command_speed.append(now_sec, float(message.twist.linear.x))
        self.command_steering.append(now_sec, float(message.twist.angular.z))
        self.command_brake.append(now_sec, float(message.twist.linear.z))

    def uart_frame_callback(self, message: String) -> None:
        self.latest_uart_frame = message.data.strip() or "<empty frame>"

    def uart_stale_callback(self, message: Bool) -> None:
        self.latest_uart_stale = bool(message.data)

    def measured_speed_callback(self, message: Float64) -> None:
        self.measured_speed.append(time.monotonic(), float(message.data))

    def measured_steering_callback(self, message: Float64) -> None:
        self.measured_steering.append(time.monotonic(), float(message.data))

    def drive_mode_callback(self, message: Int32) -> None:
        self.latest_drive_mode = int(message.data)

    def emergency_stop_callback(self, message: Bool) -> None:
        self.latest_emergency_stop = bool(message.data)

    def prune_buffers(self, now_sec: float) -> None:
        self.command_speed.prune(now_sec, self.window_sec)
        self.command_steering.prune(now_sec, self.window_sec)
        self.command_brake.prune(now_sec, self.window_sec)
        self.measured_speed.prune(now_sec, self.window_sec)
        self.measured_steering.prune(now_sec, self.window_sec)
        self._prune_scan_times(now_sec)

    def scan_status(self, now_sec: float) -> str:
        if self.last_scan_time is None:
            return "waiting"
        scan_age = now_sec - self.last_scan_time
        return f"{scan_age:5.2f}s ago, {self.scan_rate_hz():4.1f} Hz"

    def scan_rate_hz(self) -> float:
        if len(self.scan_arrival_times) < 2:
            return 0.0
        duration = self.scan_arrival_times[-1] - self.scan_arrival_times[0]
        if duration <= 0.0:
            return 0.0
        return (len(self.scan_arrival_times) - 1) / duration

    def status_lines(self, now_sec: float) -> List[str]:
        self.prune_buffers(now_sec)
        return [
            f"Buffer window: {self.window_sec:.0f}s sliding window (older samples pruned).",
            f"Scan topic: {self.scan_topic} | last scan: {self.scan_status(now_sec)}",
            (
                f"UART telemetry: {self.uart_telemetry_topic} | "
                f"state: {'STALE/TIMEOUT' if self.latest_uart_stale else 'fresh'}"
            ),
            f"Latest UART frame: {self.latest_uart_frame}",
            (
                f"Drive mode: {drive_mode_name(self.latest_drive_mode)} | "
                f"emergency stop: {self.latest_emergency_stop}"
            ),
            (
                f"Command speed: {format_value(self.command_speed.latest_value(), 'm/s')} | "
                f"measured speed: {format_value(self.measured_speed.latest_value(), 'm/s')}"
            ),
            (
                f"Command angular.z: {format_value(self.command_steering.latest_value(), 'rad/raw')} | "
                f"measured steering: {format_value(self.measured_steering.latest_value(), 'rad')}"
            ),
            f"Command brake field: {format_value(self.command_brake.latest_value(), 'brake')}",
            (
                f"Future measured topics: speed={self.measured_speed_topic} "
                f"steering={self.measured_steering_topic}"
            ),
        ]

    def _prune_scan_times(self, now_sec: float) -> None:
        cutoff = now_sec - 5.0
        while self.scan_arrival_times and self.scan_arrival_times[0] < cutoff:
            self.scan_arrival_times.popleft()


class SlidingPlotWidget(QtWidgets.QWidget):
    def __init__(
        self,
        node: DashboardNode,
        title: str,
        y_axis_label: str,
        command_series: TimeSeries,
        command_label: str,
        command_color: str,
        measured_series: TimeSeries,
        measured_label: str,
        measured_color: str,
        default_limits: Tuple[float, float],
    ) -> None:
        super().__init__()
        self.node = node
        self.title = title
        self.y_axis_label = y_axis_label
        self.command_series = command_series
        self.command_label = command_label
        self.command_color = QtGui.QColor(command_color)
        self.measured_series = measured_series
        self.measured_label = measured_label
        self.measured_color = QtGui.QColor(measured_color)
        self.default_limits = default_limits
        self.now_sec = time.monotonic()
        self.setMinimumHeight(220)

    def refresh(self, now_sec: float) -> None:
        self.now_sec = now_sec
        self.update()

    def paintEvent(self, _: QtGui.QPaintEvent) -> None:
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        background = QtGui.QColor("#0f172a")
        panel = QtGui.QColor("#111827")
        grid = QtGui.QColor("#334155")
        text = QtGui.QColor("#e2e8f0")
        axis = QtGui.QColor("#94a3b8")

        painter.fillRect(self.rect(), background)

        plot_rect = QtCore.QRectF(62.0, 24.0, self.width() - 86.0, self.height() - 58.0)
        if plot_rect.width() <= 0 or plot_rect.height() <= 0:
            return

        painter.fillRect(plot_rect, panel)
        painter.setPen(QtGui.QPen(grid, 1.0))
        for grid_index in range(6):
            x_value = plot_rect.left() + grid_index * plot_rect.width() / 5.0
            painter.drawLine(
                QtCore.QPointF(x_value, plot_rect.top()),
                QtCore.QPointF(x_value, plot_rect.bottom()),
            )
        for grid_index in range(5):
            y_value = plot_rect.top() + grid_index * plot_rect.height() / 4.0
            painter.drawLine(
                QtCore.QPointF(plot_rect.left(), y_value),
                QtCore.QPointF(plot_rect.right(), y_value),
            )

        command_x, command_y = self.command_series.points(self.now_sec, self.node.window_sec)
        measured_x, measured_y = self.measured_series.points(self.now_sec, self.node.window_sec)
        y_min, y_max = compute_limits((command_y, measured_y), self.default_limits)

        painter.setPen(QtGui.QPen(axis, 1.5))
        painter.drawRect(plot_rect)

        self._draw_series(
            painter,
            plot_rect,
            command_x,
            command_y,
            y_min,
            y_max,
            self.command_color,
        )
        self._draw_series(
            painter,
            plot_rect,
            measured_x,
            measured_y,
            y_min,
            y_max,
            self.measured_color,
            dashed=True,
        )

        painter.setPen(text)
        title_font = QtGui.QFont("DejaVu Sans", 11, QtGui.QFont.Bold)
        painter.setFont(title_font)
        painter.drawText(
            QtCore.QRectF(12.0, 4.0, self.width() - 24.0, 18.0),
            QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter,
            self.title,
        )

        label_font = QtGui.QFont("DejaVu Sans Mono", 9)
        painter.setFont(label_font)
        painter.drawText(
            QtCore.QRectF(8.0, plot_rect.top() - 6.0, 48.0, 16.0),
            QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter,
            f"{y_max: .2f}",
        )
        painter.drawText(
            QtCore.QRectF(8.0, plot_rect.bottom() - 10.0, 48.0, 16.0),
            QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter,
            f"{y_min: .2f}",
        )
        painter.drawText(
            QtCore.QRectF(12.0, plot_rect.center().y() - 8.0, 40.0, 16.0),
            QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter,
            self.y_axis_label,
        )

        tick_step = max(10, int(self.node.window_sec // 6) or 1)
        for tick in range(0, int(self.node.window_sec) + 1, tick_step):
            x_value = plot_rect.right() - (tick / self.node.window_sec) * plot_rect.width()
            painter.drawText(
                QtCore.QRectF(x_value - 20.0, plot_rect.bottom() + 8.0, 40.0, 16.0),
                QtCore.Qt.AlignCenter,
                f"-{tick}s" if tick > 0 else "now",
            )

        self._draw_legend(painter, plot_rect, text)

    def _draw_series(
        self,
        painter: QtGui.QPainter,
        plot_rect: QtCore.QRectF,
        x_values: Sequence[float],
        y_values: Sequence[float],
        y_min: float,
        y_max: float,
        color: QtGui.QColor,
        dashed: bool = False,
    ) -> None:
        if not x_values or not y_values:
            return

        path = QtGui.QPainterPath()
        for index, (x_value, y_value) in enumerate(zip(x_values, y_values)):
            x_pos = plot_rect.left() + ((x_value + self.node.window_sec) / self.node.window_sec) * plot_rect.width()
            if abs(y_max - y_min) < 1e-9:
                y_pos = plot_rect.center().y()
            else:
                ratio = (y_value - y_min) / (y_max - y_min)
                y_pos = plot_rect.bottom() - ratio * plot_rect.height()
            point = QtCore.QPointF(x_pos, y_pos)
            if index == 0:
                path.moveTo(point)
            else:
                path.lineTo(point)

        pen = QtGui.QPen(color, 2.2)
        if dashed:
            pen.setStyle(QtCore.Qt.DashLine)
        painter.setPen(pen)
        painter.drawPath(path)

        if len(x_values) == 1:
            painter.setBrush(color)
            painter.drawEllipse(path.currentPosition(), 3.0, 3.0)

    def _draw_legend(
        self,
        painter: QtGui.QPainter,
        plot_rect: QtCore.QRectF,
        text_color: QtGui.QColor,
    ) -> None:
        entries = [
            (self.command_color, self.command_label, False),
            (self.measured_color, self.measured_label, True),
        ]
        legend_x = plot_rect.right() - 220.0
        legend_y = plot_rect.top() + 10.0
        painter.setFont(QtGui.QFont("DejaVu Sans", 9))
        for index, (color, label, dashed) in enumerate(entries):
            y_value = legend_y + index * 18.0
            pen = QtGui.QPen(color, 2.2)
            if dashed:
                pen.setStyle(QtCore.Qt.DashLine)
            painter.setPen(pen)
            painter.drawLine(
                QtCore.QPointF(legend_x, y_value),
                QtCore.QPointF(legend_x + 20.0, y_value),
            )
            painter.setPen(text_color)
            painter.drawText(
                QtCore.QRectF(legend_x + 28.0, y_value - 8.0, 180.0, 16.0),
                QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter,
                label,
            )


class DashboardWindow(QtWidgets.QWidget):
    def __init__(self, node: DashboardNode) -> None:
        super().__init__()
        self.node = node
        self.setWindowTitle("F1TENTH Real-Car Dashboard")
        self.resize(1200, 860)

        outer_layout = QtWidgets.QVBoxLayout(self)
        outer_layout.setContentsMargins(12, 12, 12, 12)
        outer_layout.setSpacing(10)

        header = QtWidgets.QLabel("F1TENTH Real-Car Dashboard")
        header.setStyleSheet(
            "color: #e2e8f0; font-size: 20px; font-weight: 600; "
            "padding: 4px 6px; background: #0f172a;"
        )
        outer_layout.addWidget(header)

        self.speed_plot = SlidingPlotWidget(
            node=self.node,
            title="Live Longitudinal Speed",
            y_axis_label="m/s",
            command_series=self.node.command_speed,
            command_label="UART command speed",
            command_color="#38bdf8",
            measured_series=self.node.measured_speed,
            measured_label="Measured speed",
            measured_color="#f59e0b",
            default_limits=(-0.1, 2.0),
        )
        outer_layout.addWidget(self.speed_plot)

        self.steering_plot = SlidingPlotWidget(
            node=self.node,
            title="Live Steering Template",
            y_axis_label="rad",
            command_series=self.node.command_steering,
            command_label="UART angular.z field",
            command_color="#34d399",
            measured_series=self.node.measured_steering,
            measured_label="Measured steering",
            measured_color="#f87171",
            default_limits=(-0.7, 0.7),
        )
        outer_layout.addWidget(self.steering_plot)

        self.status_box = QtWidgets.QPlainTextEdit()
        self.status_box.setReadOnly(True)
        self.status_box.setMaximumBlockCount(100)
        self.status_box.setFixedHeight(180)
        self.status_box.setStyleSheet(
            "background: #111827; color: #e2e8f0; border: 1px solid #334155; "
            "font-family: 'DejaVu Sans Mono'; font-size: 12px;"
        )
        outer_layout.addWidget(self.status_box)

        self.setStyleSheet("background: #020617;")

    def refresh(self) -> None:
        now_sec = time.monotonic()
        self.node.prune_buffers(now_sec)
        self.speed_plot.refresh(now_sec)
        self.steering_plot.refresh(now_sec)
        self.status_box.setPlainText("\n".join(self.node.status_lines(now_sec)))


def drive_mode_name(mode: Optional[int]) -> str:
    return {
        None: "waiting",
        0: "LOCKED",
        1: "MANUAL",
        2: "AUTONOMOUS",
    }.get(mode, f"UNKNOWN({mode})")


def format_value(value: Optional[float], unit: str) -> str:
    if value is None:
        return "waiting"
    return f"{value: .3f} {unit}"


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
