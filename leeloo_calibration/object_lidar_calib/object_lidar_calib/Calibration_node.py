#!/usr/bin/env python3

import math

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger
import tf2_ros
from tf2_ros import TransformException
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from object_lidar_calib_msgs.msg import PrismDetection


class CalibrationNode(Node):
    """Compute the prism center from two captures on different corners.

    Workflow:
      1. Position the robot in front of one corner (110° or 135°).
      2. Call service 'capture_calibration_data' — capture 1/2.
      3. Move the robot to face the other corner.
      4. Call the service again — capture 2/2.
         → The static TF 'prism_center' is published in 'odom'.

    Geometry:
      - 110° corner: offset OFFSET_M along the LiDAR X axis (in odom).
      - 135° corner: offset OFFSET_M along the LiDAR Y axis (in odom).

    TF architecture:
      - The robot publishes its transforms on /r100_0597/tf[_static].
        This node subscribes and injects them into its buffer via set_transform(),
        without relying on an external relay.
      - 'odom → base_link' is read directly from the odometry topic
        (more reliable than via the TF buffer for calibration).
    """

    LIDAR_FRAME       = 'lidar2d_0_laser'
    BASE_FRAME        = 'base_link'
    ODOM_FRAME        = 'odom'
    TOOL_CENTER_FRAME = 'prism_center'
    OFFSET_M          = -0.085   # décalage centre↔sommet, sens négatif de l'axe LiDAR

    ODOM_TOPIC = '/r100_0597/platform/odom/filtered'
    # Topics TF du robot (namespace propre, séparés du /tf global)
    ROBOT_TF_TOPIC        = '/r100_0597/tf'
    ROBOT_TF_STATIC_TOPIC = '/r100_0597/tf_static'

    def __init__(self):
        super().__init__('calibration_node')

        # ── Merged TF buffer ──────────────────────────────────────────────────
        # The robot publishes on /r100_0597/tf[_static], prism_detector_node on
        # /tf[_static]. Both are injected into the same buffer so the full chain
        # base_link → ... → prism_tool can be resolved.
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS matching TransformListener for /tf_static:
        # TRANSIENT_LOCAL to receive transforms published before node startup.
        static_qos = QoSProfile(
            depth=100,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            TFMessage, self.ROBOT_TF_TOPIC,
            self._robot_tf_callback, 100
        )
        self.create_subscription(
            TFMessage, self.ROBOT_TF_STATIC_TOPIC,
            self._robot_tf_static_callback, static_qos
        )
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # ── Subscriptions ────────────────────────────────────────────────────
        self.create_subscription(
            PrismDetection, '/prism_detection', self._detection_callback, 10
        )
        self.create_subscription(
            Odometry, self.ODOM_TOPIC, self._odom_callback, 10
        )
        self.create_service(
            Trigger, 'capture_calibration_data', self._trigger_callback
        )

        # ── Internal state ───────────────────────────────────────────────────
        self._latest_detection = None   # most recent PrismDetection
        self._latest_odom      = None   # most recent Odometry
        self._first_capture    = None   # dict {'corner': int, 'center': np.ndarray}
        self._calibration_done = False

        self.get_logger().info(
            "Nœud de calibration démarré. "
            "Appelez 'capture_calibration_data' deux fois (coins différents) "
            "pour calculer le centre de l'outil."
        )

    # ── TF tree merging ───────────────────────────────────────────────────────

    def _robot_tf_callback(self, msg):
        for transform in msg.transforms:
            self.tf_buffer.set_transform(transform, 'robot_tf')

    def _robot_tf_static_callback(self, msg):
        for transform in msg.transforms:
            self.tf_buffer.set_transform_static(transform, 'robot_tf_static')

    # ── Callbacks ────────────────────────────────────────────────────────────

    def _detection_callback(self, msg):
        self._latest_detection = msg

    def _odom_callback(self, msg):
        self._latest_odom = msg

    def _trigger_callback(self, request, response):
        if self._calibration_done:
            response.success = False
            response.message = (
                "Calibration déjà terminée. "
                "Redémarrez le nœud pour recommencer."
            )
            return response

        if self._latest_detection is None:
            response.success = False
            response.message = "ÉCHEC : aucun angle du prisme détecté sur /prism_detection."
            self.get_logger().warn(response.message)
            return response

        if self._latest_odom is None:
            response.success = False
            response.message = f"ÉCHEC : pas de données d'odométrie sur {self.ODOM_TOPIC}."
            self.get_logger().warn(response.message)
            return response

        corner_type = self._latest_detection.detected_corner  # 110 ou 135

        center_odom = self._compute_center_in_odom(corner_type)
        if center_odom is None:
            response.success = False
            response.message = (
                f"ÉCHEC : impossible de calculer le centre pour le coin {corner_type}°."
            )
            return response

        if self._first_capture is None:
            self._first_capture = {'corner': corner_type, 'center': center_odom}
            response.success = True
            response.message = (
                f"Capture 1/2 — coin {corner_type}° : "
                f"centre estimé à ({center_odom[0]:.4f}, {center_odom[1]:.4f}, {center_odom[2]:.4f}) m dans odom. "
                "Déplacez le robot pour voir l'autre coin."
            )
            self.get_logger().info(response.message)

        else:
            if corner_type == self._first_capture['corner']:
                response.success = False
                response.message = (
                    f"ÉCHEC : le même coin ({corner_type}°) est encore visible. "
                    "Déplacez le robot pour voir l'autre coin."
                )
                self.get_logger().warn(response.message)
                return response

            c1 = self._first_capture['center']
            c2 = center_odom
            center_final = (c1 + c2) / 2.0
            delta_cm = float(np.linalg.norm(c1 - c2)) * 100.0

            self._publish_center_tf(center_final)
            self._calibration_done = True

            response.success = True
            response.message = (
                f"Capture 2/2 — coin {corner_type}°. "
                f"Centre de l'outil publié dans '{self.ODOM_FRAME}' : "
                f"x={center_final[0]:.4f} m, y={center_final[1]:.4f} m, z={center_final[2]:.4f} m. "
                f"Écart XY entre les deux estimées : {delta_cm:.1f} cm."
            )
            self.get_logger().info(response.message)

        return response

    # ── Geometry ──────────────────────────────────────────────────────────────

    def _compute_center_in_odom(self, corner_type):
        """Return the tool center position in odom.

        Steps:
          1. Lookup 'prism_tool' in 'base_link' via the TF buffer.
             (chain: base_link → chassis_link → lidar2d_0_link →
              lidar2d_0_laser → prism_tool — fully in /tf and /tf_static)
          2. Lookup 'lidar2d_0_laser' in 'base_link' (static, always available).
          3. Read odom → base_link pose from the odometry message.
          4. Transform the apex from base_link to odom.
          5. Compute the LiDAR axes in odom and apply the offset.

        Returns:
            np.ndarray (3,) — (x, y, z) in odom — or None if a lookup fails.
        """
        # 1. Corner apex in base_link
        try:
            tf_corner = self.tf_buffer.lookup_transform(
                self.BASE_FRAME, 'prism_tool', rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().error(f"Lookup prism_tool → {self.BASE_FRAME} échoué : {ex}")
            return None

        # 2. LiDAR orientation in base_link (static)
        try:
            tf_lidar = self.tf_buffer.lookup_transform(
                self.BASE_FRAME, self.LIDAR_FRAME, rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().error(
                f"Lookup {self.LIDAR_FRAME} → {self.BASE_FRAME} échoué : {ex}"
            )
            return None

        # 3. odom → base_link pose from odometry
        odom_pos  = self._latest_odom.pose.pose.position
        odom_quat = self._latest_odom.pose.pose.orientation
        odom_yaw  = 2.0 * math.atan2(odom_quat.z, odom_quat.w)
        cos_o = math.cos(odom_yaw)
        sin_o = math.sin(odom_yaw)

        # 4. Apex in odom (yaw-only rotation on X/Y; Z kept as robot is on flat ground)
        cx = tf_corner.transform.translation.x
        cy = tf_corner.transform.translation.y
        cz = tf_corner.transform.translation.z
        corner_in_odom = np.array([
            odom_pos.x + cos_o * cx - sin_o * cy,
            odom_pos.y + sin_o * cx + cos_o * cy,
            odom_pos.z + cz,
        ])

        # 5. LiDAR axes in odom
        q_lidar = tf_lidar.transform.rotation
        lidar_yaw_in_base = 2.0 * math.atan2(q_lidar.z, q_lidar.w)
        lidar_yaw_in_odom = odom_yaw + lidar_yaw_in_base
        lidar_x_in_odom = np.array([ math.cos(lidar_yaw_in_odom), math.sin(lidar_yaw_in_odom)])
        lidar_y_in_odom = np.array([-math.sin(lidar_yaw_in_odom), math.cos(lidar_yaw_in_odom)])

        if corner_type == 110:
            xy = corner_in_odom[:2] + self.OFFSET_M * lidar_x_in_odom
        elif corner_type == 135:
            xy = corner_in_odom[:2] + self.OFFSET_M * lidar_y_in_odom
        else:
            self.get_logger().error(f"Type de coin non supporté : {corner_type}°")
            return None

        return np.array([xy[0], xy[1], corner_in_odom[2]])

    def _publish_center_tf(self, center_odom):
        """Publish the static TF of the tool center in the odom frame.

        Orientation: 90° rotation around Z from identity.
        Z: same as prism_tool (LiDAR height in odom).
        """
        yaw_90 = math.pi / 2.0
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.ODOM_FRAME
        t.child_frame_id = self.TOOL_CENTER_FRAME
        t.transform.translation.x = float(center_odom[0])
        t.transform.translation.y = float(center_odom[1])
        t.transform.translation.z = float(center_odom[2])
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(yaw_90 / 2.0)   # sin(π/4) ≈ 0.7071
        t.transform.rotation.w = math.cos(yaw_90 / 2.0)   # cos(π/4) ≈ 0.7071
        self.tf_static_broadcaster.sendTransform(t)
        self.get_logger().info(
            f"TF statique '{self.TOOL_CENTER_FRAME}' publié dans '{self.ODOM_FRAME}' : "
            f"x={center_odom[0]:.4f} m, y={center_odom[1]:.4f} m, z={center_odom[2]:.4f} m, "
            f"yaw=90°."
        )


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
