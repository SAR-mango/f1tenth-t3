import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


DEFAULT_SPEED_SEGMENTS = [
    (0.5, 12.0),
    (0.6, 10.0),
    (0.75, 8.0),
    (1.0, 6.0),
    (1.5, 4.0),
    (2.0, 3.0),
    (3.0, 2.0),
]


class SpeedTestNode(Node):
    """Wait, run the requested speed-and-dwell schedule, then remain stopped."""

    def __init__(self):
        super().__init__("speed_test")

        self.startup_delay_sec = float(self.declare_parameter("startup_delay_sec", 3.0).value)
        self.inter_step_wait_sec = float(self.declare_parameter("inter_step_wait_sec", 20.0).value)
        self.control_rate_hz = float(self.declare_parameter("control_rate_hz", 20.0).value)
        self.cmd_vel_topic = str(self.declare_parameter("cmd_vel_topic", "/cmd_vel").value)

        self._speed_segments = [
            (float(speed_mps), float(duration_sec))
            for speed_mps, duration_sec in DEFAULT_SPEED_SEGMENTS
        ]
        self._phases = self._build_phases()
        self._boot_wall_time = time.monotonic()
        self._active_phase_index = None
        self._done_logged = False

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(1.0 / max(self.control_rate_hz, 1.0), self._control_loop)

        total_duration_sec = self.startup_delay_sec + sum(
            phase["duration_sec"] for phase in self._phases
        )
        drive_summary = ", ".join(
            "%.2f m/s for %.0fs" % (speed_mps, duration_sec)
            for speed_mps, duration_sec in self._speed_segments
        )
        self.get_logger().info(
            "Speed test ready: wait %.2fs, then drive straight on %s with sequence %s. "
            "Insert %.2fs stopped dwell periods between speed commands. Total timeline %.2fs."
            % (
                self.startup_delay_sec,
                self.cmd_vel_topic,
                drive_summary,
                self.inter_step_wait_sec,
                total_duration_sec,
            )
        )

    def _build_phases(self):
        phases = []
        for index, (speed_mps, duration_sec) in enumerate(self._speed_segments):
            phases.append(
                {
                    "kind": "drive",
                    "speed_mps": speed_mps,
                    "duration_sec": max(duration_sec, 0.0),
                }
            )
            if index < len(self._speed_segments) - 1:
                phases.append(
                    {
                        "kind": "wait",
                        "speed_mps": 0.0,
                        "duration_sec": max(self.inter_step_wait_sec, 0.0),
                    }
                )
        return phases

    def _publish_cmd(self, linear_x: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)

    def _lookup_phase(self, elapsed_sequence_sec: float):
        cumulative_sec = 0.0
        for phase_index, phase in enumerate(self._phases):
            cumulative_sec += phase["duration_sec"]
            if elapsed_sequence_sec < cumulative_sec:
                return phase_index, phase
        return None, None

    def _control_loop(self):
        elapsed_since_boot = time.monotonic() - self._boot_wall_time

        if elapsed_since_boot < self.startup_delay_sec:
            self._publish_cmd(0.0)
            return

        elapsed_sequence_sec = elapsed_since_boot - self.startup_delay_sec
        phase_index, phase = self._lookup_phase(elapsed_sequence_sec)

        if phase is None:
            self._publish_cmd(0.0)
            if not self._done_logged:
                self.get_logger().info("Speed test sequence complete. Remaining stopped.")
                self._done_logged = True
            return

        if phase_index != self._active_phase_index:
            self._active_phase_index = phase_index
            if phase["kind"] == "drive":
                drive_step_index = (phase_index // 2) + 1
                self.get_logger().info(
                    "Entering drive step %d/%d: straight at %.2f m/s for %.2fs."
                    % (
                        drive_step_index,
                        len(self._speed_segments),
                        phase["speed_mps"],
                        phase["duration_sec"],
                    )
                )
            else:
                previous_speed_mps = self._speed_segments[phase_index // 2][0]
                self.get_logger().info(
                    "Drive step complete. Stopping for %.2fs before the next command "
                    "(previous speed %.2f m/s)."
                    % (phase["duration_sec"], previous_speed_mps)
                )

        self._publish_cmd(phase["speed_mps"])

    def destroy_node(self):
        self._publish_cmd(0.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = SpeedTestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
