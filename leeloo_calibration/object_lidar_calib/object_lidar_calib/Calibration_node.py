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
    """Compute the prism center from two captures at different corners.

    Workflow:
      1. Position the robot in front of a corner (110° or 135°).
      2. Call the 'capture_calibration_data' service — capture 1/2.
      3. Move the robot in front of the other corner.
      4. Call the service again — capture 2/2.
         → The static TF 'prism_center' is published in 'odom'.

    Geometry (intersection method):
      The prism center is the intersection of two lines in the odom XY plane:
        - Line 1 (110° capture): passes through the detected apex along the LiDAR X axis.
        - Line 2 (135° capture): passes through the detected apex along the LiDAR Y axis.
      No fixed offset constant is required — the geometry is self-consistent.

    TF architecture:
      - The robot publishes its transforms on /r100_0597/tf[_static].
        This node subscribes to them and injects them into its buffer via set_transform(),
        without depending on an external relay.
      - 'odom → base_link' is read directly from the odometry topic
        (more reliable than via the TF buffer for calibration).
    """

    BASE_FRAME        = 'base_link'
    ODOM_FRAME        = 'odom'
    TOOL_CENTER_FRAME = 'prism_center'

    ODOM_TOPIC = '/r100_0597/platform/odom/filtered'
    # Robot TF topics (own namespace, separate from /tf global)
    ROBOT_TF_TOPIC        = '/r100_0597/tf'
    ROBOT_TF_STATIC_TOPIC = '/r100_0597/tf_static'

    def __init__(self):
        super().__init__('calibration_node')

        # ── Merged TF buffer ──────────────────────────────────────────────────
        # The robot publishes on /r100_0597/tf[_static], prism_detector_node on
        # /tf[_static]. Both are injected into the same buffer so the full chain
        # base_link → ... → prism_tool is resolvable.
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # QoS matching the one used by TransformListener for /tf_static:
        # TRANSIENT_LOCAL to receive transforms already published before startup.
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
        self._latest_detection = None   # Most recent PrismDetection
        self._latest_odom      = None   # Most recent Odometry
        # dict {'corner': int, 'apex': np.ndarray, 'direction': np.ndarray}
        self._first_capture    = None
        self._calibration_done = False

        self.get_logger().info(
            "Calibration node started. "
            "Call 'capture_calibration_data' twice (different corners) "
            "to compute the tool center."
        )

    # ── TF tree merging ───────────────────────────────────────────────────────

    def _robot_tf_callback(self, msg):
        for transform in msg.transforms:
            self.tf_buffer.set_transform(transform, 'robot_tf')

    def _robot_tf_static_callback(self, msg):
        for transform in msg.transforms:
            self.tf_buffer.set_transform_static(transform, 'robot_tf_static')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _detection_callback(self, msg):
        self._latest_detection = msg

    def _odom_callback(self, msg):
        self._latest_odom = msg

    def _trigger_callback(self, request, response):
        if self._calibration_done:
            response.success = False
            response.message = (
                "Calibration already done. "
                "Restart the node to start over."
            )
            return response

        if self._latest_detection is None:
            response.success = False
            response.message = "FAILURE: no prism corner detected on /prism_detection."
            self.get_logger().warn(response.message)
            return response

        if self._latest_odom is None:
            response.success = False
            response.message = f"FAILURE: no odometry data on {self.ODOM_TOPIC}."
            self.get_logger().warn(response.message)
            return response

        corner_type = self._latest_detection.detected_corner  # 110 or 135

        result = self._compute_apex_and_direction_in_odom(corner_type)
        if result is None:
            response.success = False
            response.message = (
                f"FAILURE: unable to compute apex/direction for corner {corner_type}°."
            )
            return response
        apex_odom, direction_odom = result

        if self._first_capture is None:
            self._first_capture = {
                'corner': corner_type,
                'apex': apex_odom,
                'direction': direction_odom,
            }
            self._publish_corner_tf(corner_type, apex_odom)
            response.success = True
            response.message = (
                f"Capture 1/2 — corner {corner_type}°: "
                f"apex at ({apex_odom[0]:.4f}, {apex_odom[1]:.4f}, {apex_odom[2]:.4f}) m in odom. "
                "Move the robot to see the other corner."
            )
            self.get_logger().info(response.message)

        else:
            if corner_type == self._first_capture['corner']:
                response.success = False
                response.message = (
                    f"FAILURE: the same corner ({corner_type}°) is still visible. "
                    "Move the robot to see the other corner."
                )
                self.get_logger().warn(response.message)
                return response

            center_final = self._compute_center_from_intersection(
                self._first_capture['apex'],      self._first_capture['direction'],
                apex_odom,                        direction_odom,
            )
            if center_final is None:
                response.success = False
                response.message = (
                    "FAILURE: the two measurement lines are parallel — "
                    "cannot compute intersection."
                )
                self.get_logger().error(response.message)
                return response

            self._publish_corner_tf(corner_type, apex_odom)
            self._publish_center_tf(center_final)
            self._calibration_done = True

            response.success = True
            response.message = (
                f"Capture 2/2 — corner {corner_type}°. "
                f"Tool center published in '{self.ODOM_FRAME}' "
                f"(intersection of LiDAR axes): "
                f"x={center_final[0]:.4f} m, y={center_final[1]:.4f} m, "
                f"z={center_final[2]:.4f} m."
            )
            self.get_logger().info(response.message)

        return response

    # ── Geometry ──────────────────────────────────────────────────────────────

    def _compute_apex_and_direction_in_odom(self, corner_type):
        """Return the apex position and the bisector direction in odom.

        Steps:
          1. Look up 'prism_tool' in 'base_link' via the global TF buffer.
             (chain: base_link → chassis_link → lidar2d_0_link →
              lidar2d_0_laser → prism_tool — entirely in /tf and /tf_static)
          2. Read the odom → base_link pose from the odometry message.
          3. Transform the apex from base_link to odom.
          4. Extract the bisector direction from the prism_tool orientation.

        The bisector direction (apex → prism center) is encoded in 'prism_tool'
        by prism_detector_node:
          - 110° corner: center lies along prism_tool X axis (yaw = bisector angle)
          - 135° corner: center lies along prism_tool Y axis (yaw = bisector angle − 90°)

        Returns:
            (apex_odom, direction_odom) where:
              apex_odom      — np.ndarray (3,): apex in odom, matches 'prism_tool'
                               at capture time.
              direction_odom — np.ndarray (2,): unit vector in odom XY pointing from
                               the apex toward the prism center.
            Returns None if a TF lookup fails.
        """
        # 1. Apex position and prism_tool orientation in base_link
        try:
            tf_corner = self.tf_buffer.lookup_transform(
                self.BASE_FRAME, 'prism_tool', rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().error(f"Lookup prism_tool → {self.BASE_FRAME} failed: {ex}")
            return None

        # 2. Pose odom → base_link from odometry
        odom_pos  = self._latest_odom.pose.pose.position
        odom_quat = self._latest_odom.pose.pose.orientation
        odom_yaw  = 2.0 * math.atan2(odom_quat.z, odom_quat.w)
        cos_o = math.cos(odom_yaw)
        sin_o = math.sin(odom_yaw)

        # 3. Apex in odom (yaw-only rotation, robot on flat ground)
        cx = tf_corner.transform.translation.x
        cy = tf_corner.transform.translation.y
        cz = tf_corner.transform.translation.z
        apex_odom = np.array([
            odom_pos.x + cos_o * cx - sin_o * cy,
            odom_pos.y + sin_o * cx + cos_o * cy,
            odom_pos.z + cz,
        ])

        # 4. Bisector direction in odom, derived from prism_tool orientation.
        #    prism_tool yaw in base_link already includes the LiDAR mounting yaw
        #    (TF chain: lidar2d_0_laser → prism_tool → ... → base_link).
        #    Composing with odom_yaw gives the tool yaw in odom.
        q_tool = tf_corner.transform.rotation
        yaw_tool_in_base = 2.0 * math.atan2(q_tool.z, q_tool.w)
        yaw_tool_in_odom = odom_yaw + yaw_tool_in_base

        if corner_type == 110:
            # Center along prism_tool X axis (yaw_tool = bisector angle)
            direction_odom = np.array([math.cos(yaw_tool_in_odom),
                                       math.sin(yaw_tool_in_odom)])
        elif corner_type == 135:
            # Center along prism_tool Y axis (yaw_tool = bisector angle − 90°)
            direction_odom = np.array([-math.sin(yaw_tool_in_odom),
                                        math.cos(yaw_tool_in_odom)])
        else:
            self.get_logger().error(f"Unsupported corner type: {corner_type}°")
            return None

        return apex_odom, direction_odom

    def _compute_center_from_intersection(self, apex1, dir1, apex2, dir2):
        """Find the prism center as the intersection of two lines in odom XY.

        Line 1: apex1 + t * dir1  (LiDAR X axis from the 110° capture)
        Line 2: apex2 + s * dir2  (LiDAR Y axis from the 135° capture)

        Solves the 2×2 linear system:
            t * dir1 - s * dir2 = apex2 - apex1

        Returns:
            np.ndarray (3,) — intersection point in odom, Z averaged from both
            apices — or None if the lines are parallel (det ≈ 0).
        """
        A = np.array([[dir1[0], -dir2[0]],
                      [dir1[1], -dir2[1]]])
        if abs(np.linalg.det(A)) < 1e-8:
            return None

        b = apex2[:2] - apex1[:2]
        t, s = np.linalg.solve(A, b)

        xy = apex1[:2] + t * dir1
        z  = (apex1[2] + apex2[2]) / 2.0

        self.get_logger().info(
            f"Intersection: t={t:.4f} m from apex1, s={s:.4f} m from apex2 "
            f"(these are the apex→center distances along each LiDAR axis)."
        )

        return np.array([xy[0], xy[1], z])

    def _publish_corner_tf(self, corner_type, apex_odom):
        """Publish a static TF freezing the detected apex for one corner capture.

        Frame name: 'prism_corner_110' or 'prism_corner_135'.
        Position:   apex of the detected corner in odom — coincides with 'prism_tool'
                    at the moment the capture was triggered.
        Orientation: identity (position-only reference marker).

        Note: StaticTransformBroadcaster re-broadcasts all previously sent transforms
        alongside the new one, so earlier corner TFs are not lost.
        """
        frame_id = f'prism_corner_{corner_type}'
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.ODOM_FRAME
        t.child_frame_id = frame_id
        t.transform.translation.x = float(apex_odom[0])
        t.transform.translation.y = float(apex_odom[1])
        t.transform.translation.z = float(apex_odom[2])
        t.transform.rotation.w = 1.0
        self.tf_static_broadcaster.sendTransform(t)
        self.get_logger().info(
            f"Static TF '{frame_id}' (apex) published in '{self.ODOM_FRAME}': "
            f"x={apex_odom[0]:.4f} m, y={apex_odom[1]:.4f} m, z={apex_odom[2]:.4f} m."
        )

    def _publish_center_tf(self, center_odom):
        """Publish the static TF of the tool center in the odom frame.

        Orientation: 90° rotation around Z relative to identity.
        Z:           average of the two apex heights in odom.
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
            f"Static TF '{self.TOOL_CENTER_FRAME}' published in '{self.ODOM_FRAME}': "
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
