import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


DEFAULT_TURNING_RADII_M = [
    0.75,
    1.0,
    1.5,
    2.0,
    3.0,
]


class SteeringTestNode(Node):
    """Wait, step through open-loop turning-radius commands, then remain stopped."""

    def __init__(self):
        super().__init__("steering_test")

        self.startup_delay_sec = float(self.declare_parameter("startup_delay_sec", 3.0).value)
        self.linear_speed_mps = float(self.declare_parameter("linear_speed_mps", 0.6).value)
        self.step_duration_sec = float(self.declare_parameter("step_duration_sec", 20.0).value)
        self.turn_right = bool(self.declare_parameter("turn_right", False).value)
        self.control_rate_hz = float(self.declare_parameter("control_rate_hz", 20.0).value)
        self.cmd_vel_topic = str(self.declare_parameter("cmd_vel_topic", "/cmd_vel").value)

        self._turning_radii_m = [abs(float(radius)) for radius in DEFAULT_TURNING_RADII_M]
        self._boot_wall_time = time.monotonic()
        self._active_step_index = None
        self._done_logged = False

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(1.0 / max(self.control_rate_hz, 1.0), self._control_loop)

        total_duration_sec = self.startup_delay_sec + (
            self.step_duration_sec * len(self._turning_radii_m)
        )
        turn_direction = "right" if self.turn_right else "left"
        self.get_logger().info(
            "Steering test ready: wait %.2fs, then %d radius steps of %.2fs each on %s. "
            "Driving %.2f m/s while turning %s with radii: %s m. Total timeline %.2fs."
            % (
                self.startup_delay_sec,
                len(self._turning_radii_m),
                self.step_duration_sec,
                self.cmd_vel_topic,
                self.linear_speed_mps,
                turn_direction,
                ", ".join("%.2f" % radius for radius in self._turning_radii_m),
                total_duration_sec,
            )
        )

        if self.step_duration_sec <= 0.0:
            self.get_logger().warn("step_duration_sec <= 0.0; the steering sequence will not run.")

    def _publish_cmd(self, linear_x: float, turning_radius_m: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(turning_radius_m)
        self.cmd_pub.publish(msg)

    def _signed_radius(self, radius_m: float) -> float:
        return -radius_m if self.turn_right else radius_m

    def _control_loop(self):
        elapsed_since_boot = time.monotonic() - self._boot_wall_time

        if elapsed_since_boot < self.startup_delay_sec:
            self._publish_cmd(0.0, 0.0)
            return

        if self.step_duration_sec <= 0.0:
            self._publish_cmd(0.0, 0.0)
            if not self._done_logged:
                self.get_logger().info("Steering test skipped because step_duration_sec <= 0.0.")
                self._done_logged = True
            return

        elapsed_sequence_sec = elapsed_since_boot - self.startup_delay_sec
        step_index = int(elapsed_sequence_sec // self.step_duration_sec)

        if step_index >= len(self._turning_radii_m):
            self._publish_cmd(0.0, 0.0)
            if not self._done_logged:
                self.get_logger().info("Steering test sequence complete. Remaining stopped.")
                self._done_logged = True
            return

        commanded_radius = self._turning_radii_m[step_index]
        if step_index != self._active_step_index:
            self._active_step_index = step_index
            self.get_logger().info(
                "Entering steering step %d/%d: %.2f m/s with %s radius %.2fm for %.2fs."
                % (
                    step_index + 1,
                    len(self._turning_radii_m),
                    self.linear_speed_mps,
                    "right" if self.turn_right else "left",
                    commanded_radius,
                    self.step_duration_sec,
                )
            )

        self._publish_cmd(self.linear_speed_mps, self._signed_radius(commanded_radius))

    def destroy_node(self):
        self._publish_cmd(0.0, 0.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = SteeringTestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
