#!/usr/bin/env python3
"""Stand-in for the real Doosan driver (dsr_controller2) in simulation.

The real arm chain is, end to end:

    curobo_ros (JointSpeedStrategy, planner_type: mpc)
      -> /leeloo/execute_trajectory            (trajectory_msgs/JointTrajectory)
    leeloo's execute_trajectory node
      -> /dsr01/dsr_controller2/speedj_rt_stream  (dsr_msgs2/SpeedjRtStream)
    dsr_controller2  ->  DRFL speedj_rt(vel, acc, time)  ->  hardware

This node replaces only the LAST step, so everything above it runs unmodified in
simulation — including execute_trajectory's dt barrier (its `command_period`
must match curobo's `interpolation_dt`), its acceleration clamp and its
zero-velocity stop path. That is the whole point of the sim: exercising the real
code, not a reimplementation of it.

Two directions:

  * command  — SpeedjRtStream.vel is in DEGREES/s (execute_trajectory.cpp
    multiplies by 180/pi on the way out); ros2_control wants rad/s. Converted
    here and forwarded to arm_velocity_controller as a Float64MultiArray.
  * feedback — curobo's doosan_m1013 descriptor reads joint feedback from
    /dsr01/joint_states. The sim's joint_state_broadcaster only publishes the
    combined /joint_states (arm + gripper), and Jazzy's broadcaster cannot be
    told to publish anywhere else, so this node republishes just the 6 arm
    joints under the topic name the descriptor expects — which is also what the
    real driver does (it publishes its own joints only).

Fidelity limits, deliberately accepted:

  * SpeedjRtStream.acc and .time are IGNORED. On hardware the DRCF uses them to
    ramp between velocity setpoints; gz_ros2_control applies a velocity command
    directly. Sim motion is therefore slightly more aggressive at MPC batch
    boundaries than the real arm's. The acceleration clamp upstream
    (curobo's max_joint_accel_dps, and execute_trajectory's own
    min/max_acceleration_dps2) still applies, so this does not hide a
    limit-violating command — it just does not reproduce the DRCF's smoothing.
  * No joint/velocity limit alarms, no Doosan alarm codes, no safety stop.
"""
import math

from dsr_msgs2.msg import SpeedjRtStream
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

DEFAULT_ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


class DsrSimBridge(Node):

    def __init__(self):
        super().__init__("dsr_sim_bridge")

        self.arm_joints = self.declare_parameter(
            "arm_joints", DEFAULT_ARM_JOINTS).value
        speedj_topic = self.declare_parameter(
            "speedj_topic", "/dsr01/dsr_controller2/speedj_rt_stream").value
        command_topic = self.declare_parameter(
            "command_topic", "/arm_velocity_controller/commands").value
        joint_states_in = self.declare_parameter(
            "joint_states_in", "/joint_states").value
        joint_states_out = self.declare_parameter(
            "joint_states_out", "/dsr01/joint_states").value

        # Watchdog. execute_trajectory streams at 1/command_period (0.03 s by
        # default), so ~3 missed ticks means the producer died, the launch was
        # torn down, or streaming was disabled via ~/enable_streaming. The real
        # DRCF stops the arm on its own when speedj_rt commands stop arriving;
        # a velocity command written into gz_ros2_control persists forever, so
        # without this the simulated arm would keep drifting after its
        # commander is gone. Same failure mode the base had before
        # base_sim_bridge — see that node's watchdog.
        self.watchdog_timeout = float(self.declare_parameter(
            "watchdog_timeout", 0.09).value)

        self._pub_command = self.create_publisher(
            Float64MultiArray, command_topic, 10)
        self._pub_joint_states = self.create_publisher(
            JointState, joint_states_out, 10)

        self._sub_speedj = self.create_subscription(
            SpeedjRtStream, speedj_topic, self._on_speedj, 10)
        self._sub_joint_states = self.create_subscription(
            JointState, joint_states_in, self._on_joint_states, 10)

        self._last_command_time = None
        self._stopped = True
        self._watchdog = self.create_timer(
            self.watchdog_timeout / 3.0, self._check_watchdog)

        self.get_logger().info(
            f"dsr_sim_bridge: {speedj_topic} (deg/s) -> {command_topic} (rad/s), "
            f"{joint_states_in} -> {joint_states_out} "
            f"(joints={self.arm_joints}, watchdog={self.watchdog_timeout}s)")

    def _on_speedj(self, msg):
        n = len(self.arm_joints)
        if len(msg.vel) < n:
            self.get_logger().warn(
                f"SpeedjRtStream carries {len(msg.vel)} velocities, expected {n}"
                " — ignoring", throttle_duration_sec=2.0)
            return

        command = Float64MultiArray()
        command.data = [math.radians(v) for v in msg.vel[:n]]
        self._pub_command.publish(command)

        self._last_command_time = self.get_clock().now()
        self._stopped = False

    def _check_watchdog(self):
        if self._stopped or self._last_command_time is None:
            return
        elapsed = (self.get_clock().now() - self._last_command_time).nanoseconds * 1e-9
        if elapsed < self.watchdog_timeout:
            return
        stop = Float64MultiArray()
        stop.data = [0.0] * len(self.arm_joints)
        self._pub_command.publish(stop)
        self._stopped = True
        self.get_logger().warn(
            f"No speedj command for {elapsed:.3f}s (> {self.watchdog_timeout}s) "
            "— commanding zero velocity")

    def _on_joint_states(self, msg):
        """Republish only the arm joints, in the descriptor's joint order."""
        index = {name: i for i, name in enumerate(msg.name)}
        try:
            picked = [index[name] for name in self.arm_joints]
        except KeyError:
            # Normal for the first few messages while controllers are still
            # activating and the broadcaster has not seen every joint yet.
            self.get_logger().warn(
                "Incoming joint_states does not carry all arm joints yet",
                throttle_duration_sec=5.0)
            return

        out = JointState()
        out.header = msg.header
        out.name = list(self.arm_joints)
        if msg.position:
            out.position = [msg.position[i] for i in picked]
        if msg.velocity:
            out.velocity = [msg.velocity[i] for i in picked]
        if msg.effort:
            out.effort = [msg.effort[i] for i in picked]
        self._pub_joint_states.publish(out)


def main():
    rclpy.init()
    node = DsrSimBridge()
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
