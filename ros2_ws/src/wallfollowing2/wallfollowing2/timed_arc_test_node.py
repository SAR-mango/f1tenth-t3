import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TimedArcTestNode(Node):
    """Wait, drive a left arc, then a right arc, then stop indefinitely."""

    def __init__(self):
        super().__init__("timed_arc_test")

        self.startup_delay_sec = float(self.declare_parameter("startup_delay_sec", 3.0).value)
        self.linear_speed_mps = float(self.declare_parameter("linear_speed_mps", 0.6).value)
        self.turn_radius_m = abs(float(self.declare_parameter("turn_radius_m", 1.0).value))
        self.left_turn_duration_sec = float(self.declare_parameter("left_turn_duration_sec", 20.0).value)
        self.right_turn_duration_sec = float(self.declare_parameter("right_turn_duration_sec", 20.0).value)
        self.control_rate_hz = float(self.declare_parameter("control_rate_hz", 40.0).value)
        self.cmd_vel_topic = str(self.declare_parameter("cmd_vel_topic", "/cmd_vel").value)

        self._boot_wall_time = time.monotonic()
        self._state = "WAIT_DELAY"
        self._state_start_wall_time = self._boot_wall_time
        self._left_turn_radius_m = self.turn_radius_m
        self._right_turn_radius_m = -self.turn_radius_m

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(1.0 / max(self.control_rate_hz, 1.0), self._control_loop)

        total_duration_sec = (
            self.startup_delay_sec + self.left_turn_duration_sec + self.right_turn_duration_sec
        )
        self.get_logger().info(
            "Timed arc test ready: wait %.2fs, then left radius %.2fm at %.2fm/s for %.2fs, "
            "then right radius %.2fm at %.2fm/s for %.2fs, then stop. Total timeline %.2fs."
            % (
                self.startup_delay_sec,
                self._left_turn_radius_m,
                self.linear_speed_mps,
                self.left_turn_duration_sec,
                abs(self._right_turn_radius_m),
                self.linear_speed_mps,
                self.right_turn_duration_sec,
                total_duration_sec,
            )
        )

    def _publish_cmd(self, linear_x: float, turning_radius_m: float):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(turning_radius_m)
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
            self._publish_cmd(0.0, 0.0)
            if now_since_boot >= self.startup_delay_sec:
                self._set_state(
                    "TURN_LEFT",
                    (
                        "Startup delay elapsed. Driving left arc at %.2fm/s with radius %.2fm."
                        % (self.linear_speed_mps, self._left_turn_radius_m)
                    ),
                )
            return

        if self._state == "TURN_LEFT":
            self._publish_cmd(self.linear_speed_mps, self._left_turn_radius_m)
            if self._elapsed_state() >= self.left_turn_duration_sec:
                self._set_state(
                    "TURN_RIGHT",
                    (
                        "Left arc complete. Driving right arc at %.2fm/s with radius %.2fm."
                        % (self.linear_speed_mps, abs(self._right_turn_radius_m))
                    ),
                )
            return

        if self._state == "TURN_RIGHT":
            self._publish_cmd(self.linear_speed_mps, self._right_turn_radius_m)
            if self._elapsed_state() >= self.right_turn_duration_sec:
                self._set_state("DONE", "Timed arc sequence complete. Remaining stopped.")
            return

        self._publish_cmd(0.0, 0.0)

    def destroy_node(self):
        self._publish_cmd(0.0, 0.0)
        super().destroy_node()


def main():
    rclpy.init()
    node = TimedArcTestNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
