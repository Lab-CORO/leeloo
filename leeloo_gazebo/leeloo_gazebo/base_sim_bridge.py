#!/usr/bin/env python3
"""Stand-in for the Ridgeback's vendor driver / firmware in simulation.

The real base chain is:

    anything publishing /cmd_vel                    (geometry_msgs/Twist)
    leeloo's ridgeback_control node
      -> /r100_0597/joy_teleop/cmd_vel              (geometry_msgs/TwistStamped)
    Clearpath firmware (on the base's own computer): wheel mixing, limits,
    and a watchdog that stops the base when commands stop arriving.

This node replaces that last step, so ridgeback_control stays in the loop
unmodified. It subscribes to ridgeback_control's real OUTPUT topic and type,
converts TwistStamped -> Twist (Gazebo's MecanumDrive plugin only accepts
gz.msgs.Twist) and republishes on the topic bridged to the plugin.

The output topic must NOT be /cmd_vel: ridgeback_control subscribes there, so
publishing to it would feed the node its own output in a loop.

Two behaviours of the real base that the bare MecanumDrive plugin does not have:

  * WATCHDOG. The plugin holds the last commanded twist forever, so a simulated
    base keeps driving after its commander dies — which is how the first cut of
    this package behaved. The Clearpath firmware stops on command timeout, and
    so does this node.
  * SPEED SATURATION. The plugin will happily track any twist it is given.
    Acceleration limiting is left to the plugin's own min_acceleration/
    max_acceleration so there is exactly one accel limiter in the chain, not two
    fighting each other.
"""
from geometry_msgs.msg import Twist, TwistStamped
import rclpy
from rclpy.node import Node


class BaseSimBridge(Node):

    def __init__(self):
        super().__init__("base_sim_bridge")

        cmd_in = self.declare_parameter(
            "cmd_vel_in", "/r100_0597/joy_teleop/cmd_vel").value
        cmd_out = self.declare_parameter(
            "cmd_vel_out", "/leeloo_gazebo/base_cmd_vel").value

        # Clearpath Ridgeback rated maximum is 1.1 m/s. The angular figure is
        # not published as clearly, so 2.0 rad/s is a representative default
        # rather than a datasheet value — override it if the real base is ever
        # characterised.
        self.max_linear = float(self.declare_parameter("max_linear_speed", 1.1).value)
        self.max_angular = float(self.declare_parameter("max_angular_speed", 2.0).value)
        self.watchdog_timeout = float(self.declare_parameter(
            "watchdog_timeout", 0.25).value)

        self._pub = self.create_publisher(Twist, cmd_out, 10)
        self._sub = self.create_subscription(
            TwistStamped, cmd_in, self._on_cmd, 10)

        self._last_command_time = None
        self._stopped = True
        self._watchdog = self.create_timer(
            self.watchdog_timeout / 3.0, self._check_watchdog)

        self.get_logger().info(
            f"base_sim_bridge: {cmd_in} (TwistStamped) -> {cmd_out} (Twist), "
            f"limits {self.max_linear} m/s / {self.max_angular} rad/s, "
            f"watchdog {self.watchdog_timeout}s")

    @staticmethod
    def _clamp(value, limit):
        return max(-limit, min(limit, value))

    def _on_cmd(self, msg):
        out = Twist()
        # linear.y is forwarded like the other axes on purpose: this base is
        # holonomic (mecanum), and ridgeback_control forwards linear.y unchanged
        # to the vendor driver. Dropping it here would silently make the sim
        # non-holonomic, which is exactly the bug this package started with.
        out.linear.x = self._clamp(msg.twist.linear.x, self.max_linear)
        out.linear.y = self._clamp(msg.twist.linear.y, self.max_linear)
        out.angular.z = self._clamp(msg.twist.angular.z, self.max_angular)
        self._pub.publish(out)

        self._last_command_time = self.get_clock().now()
        self._stopped = False

    def _check_watchdog(self):
        if self._stopped or self._last_command_time is None:
            return
        elapsed = (self.get_clock().now() - self._last_command_time).nanoseconds * 1e-9
        if elapsed < self.watchdog_timeout:
            return
        self._pub.publish(Twist())
        self._stopped = True
        self.get_logger().warn(
            f"No base command for {elapsed:.3f}s (> {self.watchdog_timeout}s) "
            "— commanding zero twist")


def main():
    rclpy.init()
    node = BaseSimBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
