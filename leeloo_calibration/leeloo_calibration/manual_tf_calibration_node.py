#!/usr/bin/env python3
"""
manual_tf_calibration_node.py
==============================
ROS2 node — package: leeloo_calibration

Lets the user manually tune the base_link → camera_base transform by
publishing float values on 6 topics. The node broadcasts the resulting TF
at 50 Hz so the user can see it live in RViz.

Topics (all std_msgs/Float64):
  /calibration_x          metres
  /calibration_y          metres
  /calibration_z          metres
  /calibration_theta_x    degrees (roll)
  /calibration_theta_y    degrees (pitch)
  /calibration_theta_z    degrees (yaw)

Service:
  /save_calibration  (std_srvs/Trigger)
      Looks up camera_base→rgb_camera_link from TF, computes
      base_link→rgb_camera_link, and writes kinect_hand_eye_result.yaml
      (the file that kinect_tf_computation_node reads).

WARNING: do NOT run kinect_tf_computation_node simultaneously —
both would publish conflicting TFs for camera_base.

Usage:
    ros2 run leeloo_calibration manual_tf_calibration_node

    ros2 topic pub /calibration_z  std_msgs/msg/Float64 "{data: 0.65}" -1
    ros2 topic pub /calibration_theta_z std_msgs/msg/Float64 "{data: 90.0}" -1
    ros2 service call /save_calibration std_srvs/srv/Trigger {}
"""

import math
import os

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64
from std_srvs.srv import Trigger

from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf_transformations
import ros2_numpy


class ManualTfCalibrationNode(Node):

    def __init__(self):
        super().__init__('manual_tf_calibration_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('parent_frame', 'base_link')
        self.declare_parameter('child_frame',  'camera_base')
        self.declare_parameter(
            'calibration_result_file',
            self._default_result_path())

        self._parent = self.get_parameter('parent_frame').value
        self._child  = self.get_parameter('child_frame').value
        self._result_file = self.get_parameter('calibration_result_file').value

        # ── Current calibration values ────────────────────────────────────────
        self._x       = 0.0
        self._y       = 0.0
        self._z       = 0.0
        self._theta_x = 0.0  # roll  (degrees)
        self._theta_y = 0.0  # pitch (degrees)
        self._theta_z = 0.0  # yaw   (degrees)

        # ── TF broadcaster ────────────────────────────────────────────────────
        self._broadcaster = TransformBroadcaster(self)

        # ── TF buffer (needed for save: look up camera_base→rgb_camera_link) ──
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(Float64, 'calibration_x',
                                 lambda m: self._set('x', m.data), 10)
        self.create_subscription(Float64, 'calibration_y',
                                 lambda m: self._set('y', m.data), 10)
        self.create_subscription(Float64, 'calibration_z',
                                 lambda m: self._set('z', m.data), 10)
        self.create_subscription(Float64, 'calibration_theta_x',
                                 lambda m: self._set('theta_x', m.data), 10)
        self.create_subscription(Float64, 'calibration_theta_y',
                                 lambda m: self._set('theta_y', m.data), 10)
        self.create_subscription(Float64, 'calibration_theta_z',
                                 lambda m: self._set('theta_z', m.data), 10)

        # ── Service ───────────────────────────────────────────────────────────
        self.create_service(Trigger, 'save_calibration', self._on_save)

        # ── Disable kinect_tf_computation_node on startup (avoid TF conflict) ─
        self._kinect_toggle_client = self.create_client(
            Trigger, '/kinect_tf_computation_node/toggle_publication')
        self._kinect_toggle_timer = self.create_timer(0.5, self._disable_kinect_tf)

        # ── Broadcast timer (50 Hz) ───────────────────────────────────────────
        self.create_timer(0.02, self._broadcast)

        self.get_logger().info(
            f'[INIT] ManualTfCalibrationNode ready.\n'
            f'  Publishing: {self._parent} → {self._child}\n'
            f'  Tune via topics: /calibration_x /calibration_y /calibration_z '
            f'/calibration_theta_x /calibration_theta_y /calibration_theta_z\n'
            f'  Save: ros2 service call /save_calibration std_srvs/srv/Trigger {{}}')

    # =========================================================================
    # Startup: disable kinect_tf_computation_node to avoid TF conflict
    # =========================================================================

    def _disable_kinect_tf(self):
        self._kinect_toggle_timer.cancel()
        if not self._kinect_toggle_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn(
                '[init] /kinect_tf_computation_node/toggle_publication not available — '
                'kinect_tf_computation_node may conflict with this node.')
            return
        future = self._kinect_toggle_client.call_async(Trigger.Request())
        future.add_done_callback(self._kinect_toggle_done)

    def _kinect_toggle_done(self, future):
        try:
            resp = future.result()
            self.get_logger().info(f'[init] kinect_tf_computation_node: {resp.message}')
        except Exception as e:
            self.get_logger().warn(f'[init] Toggle call failed: {e}')

    # =========================================================================
    # Topic callbacks
    # =========================================================================

    def _set(self, field: str, value: float):
        setattr(self, f'_{field}', value)
        self.get_logger().info(
            f'[update] {field}={value:.4f}  |  '
            f'x={self._x:.3f} y={self._y:.3f} z={self._z:.3f}  '
            f'rx={self._theta_x:.2f}° ry={self._theta_y:.2f}° rz={self._theta_z:.2f}°',
            throttle_duration_sec=0.1)

    # =========================================================================
    # TF broadcast
    # =========================================================================

    def _broadcast(self):
        roll  = math.radians(self._theta_x)
        pitch = math.radians(self._theta_y)
        yaw   = math.radians(self._theta_z)
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

        ts = TransformStamped()
        ts.header.stamp    = self.get_clock().now().to_msg()
        ts.header.frame_id = self._parent
        ts.child_frame_id  = self._child
        ts.transform.translation.x = self._x
        ts.transform.translation.y = self._y
        ts.transform.translation.z = self._z
        ts.transform.rotation.x = q[0]
        ts.transform.rotation.y = q[1]
        ts.transform.rotation.z = q[2]
        ts.transform.rotation.w = q[3]
        self._broadcaster.sendTransform(ts)

    # =========================================================================
    # Save service
    # =========================================================================

    def _on_save(self, _req, response):
        roll  = math.radians(self._theta_x)
        pitch = math.radians(self._theta_y)
        yaw   = math.radians(self._theta_z)
        q = tf_transformations.quaternion_from_euler(roll, pitch, yaw)

        # Build T_base→camera_base as 4×4
        T_base_to_camera = tf_transformations.quaternion_matrix(q)
        T_base_to_camera[0, 3] = self._x
        T_base_to_camera[1, 3] = self._y
        T_base_to_camera[2, 3] = self._z

        # Try to look up camera_base → rgb_camera_link (from Kinect URDF)
        try:
            rgb_to_cam = self._tf_buffer.lookup_transform(
                'camera_base', 'rgb_camera_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=2))
            T_camera_to_rgb = ros2_numpy.numpify(rgb_to_cam.transform)

            # T_base→rgb = T_base→camera · T_camera→rgb
            T_base_to_rgb = T_base_to_camera @ T_camera_to_rgb

            t = tf_transformations.translation_from_matrix(T_base_to_rgb)
            r = tf_transformations.quaternion_from_matrix(T_base_to_rgb)

            data = {
                'transform': {
                    'translation': {'x': float(t[0]), 'y': float(t[1]), 'z': float(t[2])},
                    'rotation':    {'x': float(r[0]), 'y': float(r[1]),
                                    'z': float(r[2]), 'w': float(r[3])},
                }
            }
            note = 'base_link → rgb_camera_link (composed with camera_base→rgb from TF)'

        except Exception as e:
            self.get_logger().warn(
                f'[save] TF lookup camera_base→rgb_camera_link failed: {e}\n'
                '  Saving base_link→camera_base directly (approximate — '
                'Kinect TF offset not applied).')

            data = {
                '_warning': 'base_link→camera_base stored directly (no camera_base→rgb TF available)',
                'transform': {
                    'translation': {'x': self._x, 'y': self._y, 'z': self._z},
                    'rotation':    {'x': float(q[0]), 'y': float(q[1]),
                                    'z': float(q[2]), 'w': float(q[3])},
                }
            }
            note = 'base_link → camera_base (approximate)'

        os.makedirs(os.path.dirname(os.path.abspath(self._result_file)), exist_ok=True)
        try:
            with open(self._result_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
            msg = f'Saved ({note}) → {self._result_file}'
            self.get_logger().info(f'[save] {msg}')
            response.success = True
            response.message = msg
        except OSError as e:
            response.success = False
            response.message = f'Write failed: {e}'

        return response

    # =========================================================================
    # Helpers
    # =========================================================================

    @staticmethod
    def _default_result_path() -> str:
        return os.path.normpath(os.path.join(
            os.path.dirname(__file__), '..', 'config', 'kinect_hand_eye_result.yaml'))


# =============================================================================
# main
# =============================================================================

def main(args=None):
    rclpy.init(args=args)
    node = ManualTfCalibrationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
