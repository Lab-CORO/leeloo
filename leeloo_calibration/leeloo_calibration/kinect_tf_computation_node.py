#!/usr/bin/env python3

import os
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger
import tf_transformations
from tf2_ros import StaticTransformBroadcaster, TransformListener, Buffer
import ros2_numpy
import numpy as np
from numpy.linalg import inv


class kinectTFComputationNode(Node):
    def __init__(self):
        super().__init__('kinect_tf_computation_node')

        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # spin_thread=False: TF messages are processed by the main rclpy.spin()
        # rather than a dedicated background thread, avoiding constant CPU use.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)

        self.declare_parameter('calibration_result_file',
            '/home/ros2_ws/src/leeloo_calibration/config/kinect_hand_eye_result.yaml')

        # base_link → rgb_camera_link loaded from hand-eye calibration YAML.
        self.base_link_to_rgb = TransformStamped()
        self.base_link_to_rgb.header.frame_id = 'base_link'
        self.base_link_to_rgb.child_frame_id = 'rgb_camera_link'
        self._load_calibration()

        # Cached camera_base ← rgb_camera_link (static, from Kinect URDF).
        # Set once on first successful TF lookup; never needs to be re-fetched.
        self._camera_tf_cached = None

        self.create_service(Trigger, '~/reload_calibration', self._reload_calibration_cb)
        self.create_service(Trigger, '~/toggle_publication', self._toggle_publication_cb)

        self._publication_enabled = True

        # Retry at 2 Hz until the Kinect robot_state_publisher has published
        # the static camera_base ← rgb_camera_link transform, then cancel.
        self._init_timer = self.create_timer(0.5, self._try_init)

    # ── Initialisation ────────────────────────────────────────────────────────

    def _try_init(self):
        """Look up the static camera TF once; publish and stop retrying on success."""
        try:
            rgb_to_cam = self.tf_buffer.lookup_transform(
                'camera_base', 'rgb_camera_link', rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(
                f'Waiting for camera_base←rgb_camera_link TF: {e}',
                throttle_duration_sec=5.0)
            return

        self._camera_tf_cached = rgb_to_cam
        self._publish_static_tf()
        self._init_timer.cancel()
        self.get_logger().info('world→camera_base static TF published.')

    # ── Publishing ────────────────────────────────────────────────────────────

    def _publish_static_tf(self):
        """Compute and broadcast the static world→camera_base transform."""
        if self._camera_tf_cached is None or not self._publication_enabled:
            return
        result_tf = self.compute_base_link_to_camera_base(
            self._camera_tf_cached, self.base_link_to_rgb)
        result_tf.header.stamp = self.get_clock().now().to_msg()
        self.tf_static_broadcaster.sendTransform(result_tf)

    # ── Calibration helpers ───────────────────────────────────────────────────

    def _reload_calibration_cb(self, request, response):
        """Reload the YAML calibration file and republish the static TF."""
        self._load_calibration()
        self._publish_static_tf()
        response.success = True
        response.message = 'Calibration rechargée.'
        return response

    def _toggle_publication_cb(self, request, response):
        """Toggle TF publication on/off."""
        self._publication_enabled = not self._publication_enabled
        state = 'ENABLED' if self._publication_enabled else 'DISABLED'
        self.get_logger().info(f'[toggle] TF publication {state}.')
        if self._publication_enabled:
            self._publish_static_tf()
        response.success = True
        response.message = f'TF publication {state}.'
        return response

    def _load_calibration(self):
        """Load base_link→rgb_camera_link from the hand-eye calibration YAML."""
        filepath = self.get_parameter('calibration_result_file').value
        if not os.path.isfile(filepath):
            self.get_logger().warn(
                f'Fichier de calibration introuvable : {filepath}. '
                'Le transform base_link→rgb_camera_link ne sera pas publié.')
            return
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)
        t = data['transform']
        self.base_link_to_rgb.transform.translation.x = t['translation']['x']
        self.base_link_to_rgb.transform.translation.y = t['translation']['y']
        self.base_link_to_rgb.transform.translation.z = t['translation']['z']
        self.base_link_to_rgb.transform.rotation.x = t['rotation']['x']
        self.base_link_to_rgb.transform.rotation.y = t['rotation']['y']
        self.base_link_to_rgb.transform.rotation.z = t['rotation']['z']
        self.base_link_to_rgb.transform.rotation.w = t['rotation']['w']
        self.get_logger().info(
            f'Calibration chargée depuis {filepath} : '
            f'tx={t["translation"]["x"]:.4f} ty={t["translation"]["y"]:.4f} '
            f'tz={t["translation"]["z"]:.4f} '
            f'qx={t["rotation"]["x"]:.4f} qy={t["rotation"]["y"]:.4f} '
            f'qz={t["rotation"]["z"]:.4f} qw={t["rotation"]["w"]:.4f}')

    # ── Computation ───────────────────────────────────────────────────────────

    def compute_base_link_to_camera_base(self, rgb_to_cam, base_link_to_rgb):
        """
        Compute world→camera_base from:
          - base_link→rgb_camera_link  (hand-eye calibration result)
          - rgb_camera_link→camera_base (static, from Kinect URDF via TF)
        """
        T_rgb_to_camera_base = ros2_numpy.numpify(rgb_to_cam.transform)
        T_base_link_to_rgb = ros2_numpy.numpify(base_link_to_rgb.transform)
        T_base_link_camera_base = np.dot(T_base_link_to_rgb, inv(T_rgb_to_camera_base))

        translation = tf_transformations.translation_from_matrix(T_base_link_camera_base)
        rotation = tf_transformations.quaternion_from_matrix(T_base_link_camera_base)

        result_tf = TransformStamped()
        result_tf.header.frame_id = 'world'
        result_tf.child_frame_id = 'camera_base'
        result_tf.transform.translation.x = translation[0]
        result_tf.transform.translation.y = translation[1]
        result_tf.transform.translation.z = translation[2]
        result_tf.transform.rotation.x = rotation[0]
        result_tf.transform.rotation.y = rotation[1]
        result_tf.transform.rotation.z = rotation[2]
        result_tf.transform.rotation.w = rotation[3]
        return result_tf


def main(args=None):
    rclpy.init(args=args)
    node = kinectTFComputationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
