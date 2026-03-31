#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from object_lidar_calib_msgs.msg import PrismDetection
import tf2_ros
import numpy as np
from sklearn.cluster import DBSCAN
import cv2
import math
from collections import deque


class PrismDetectorNode(Node):
    def __init__(self):
        super().__init__('prism_detector_node')

        self.lidar_topic = '/r100_0597/sensors/lidar2d_0/scan'

        self.scan_sub = self.create_subscription(LaserScan, self.lidar_topic, self.scan_callback, 10)
        self.detection_pub = self.create_publisher(PrismDetection, '/prism_detection', 10)
        self.filtered_scan_pub = self.create_publisher(LaserScan, '/filtered_scan', 10)

        # ── Detection parameters ─────────────────────────────────────────────
        # 160° removed: too often confused with 110° and unreliably detected
        self.target_angles = {110.0, 135.0}
        self.angle_tolerance = 8.0
        self.poly_epsilon = 0.015

        # Search zone: forward half-space only, within a reasonable range
        self.max_range = 3.0          # m — beyond this it is not the tool
        self.min_range = 0.1          # m
        self.fov_half_deg = 90.0      # ±90° around the forward axis (angle=0)

        # Apex distance constraint (tool is ~70 cm away)
        self.min_corner_dist = 0.20   # m — tool cannot overlap the robot
        self.max_corner_dist = 2.0    # m — beyond this it is the environment

        # Corner quality: arm length and balance
        self.min_arm_length = 0.04    # m — each arm must be ≥ 4 cm
        self.max_arm_length = 0.10    # m — tool is small, not a wall
        self.max_arm_ratio = 4.0      # long_arm / short_arm ≤ 4

        # Gap threshold: stop arm collection when consecutive points exceed this
        self.cluster_gap_threshold = 0.009  # m

        # ── Temporal filter ──────────────────────────────────────────────────
        # Accept a detection only if consistent with the N previous frames.
        # Published pose is the sliding average of the last K accepted detections.
        self.history_size = 5                    # frames in the sliding average
        self.max_jump = 0.25                     # max position jump between frames (m)
        self.max_yaw_jump = math.radians(30.0)  # max yaw jump — rejects outliers
        self.min_consecutive = 3                 # consecutive hits before publishing

        self._pose_history = deque(maxlen=self.history_size)
        self._consecutive_detections = 0

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info("Prism detector démarré — publication sur /prism_detection")

    # ── Main callback ────────────────────────────────────────────────────────

    def scan_callback(self, msg):
        angles = np.arange(msg.angle_min, msg.angle_max, msg.angle_increment)
        min_len = min(len(angles), len(msg.ranges))
        angles = angles[:min_len]
        ranges = np.array(msg.ranges)[:min_len]

        # 1. Basic filter: distance range + frontal FOV
        fov_rad = math.radians(self.fov_half_deg)
        valid_mask = (
            np.isfinite(ranges)
            & (ranges > self.min_range)
            & (ranges < self.max_range)
            & (np.abs(angles) < fov_rad)
        )
        valid_ranges = ranges[valid_mask]
        valid_angles = angles[valid_mask]
        original_indices = np.where(valid_mask)[0]

        x = valid_ranges * np.cos(valid_angles)
        y = valid_ranges * np.sin(valid_angles)
        points = np.vstack((x, y)).T

        if len(points) < 5:
            self._register_no_detection()
            return

        # 2. DBSCAN clustering
        clustering = DBSCAN(eps=0.08, min_samples=5).fit(points)
        labels = clustering.labels_

        best_corner = None          # (p_prev, p_curr, p_next, angle_type, score, cluster_mask)
        indices_to_keep = []

        for label in set(labels):
            if label == -1:
                continue

            cluster_mask = (labels == label)
            cluster_points = points[cluster_mask]

            if len(cluster_points) < 10:
                continue

            contour = cluster_points.reshape(-1, 1, 2).astype(np.float32)
            approx = cv2.approxPolyDP(contour, self.poly_epsilon, False)

            if len(approx) < 3:
                continue

            vertices = approx.reshape(-1, 2)

            for i in range(1, len(vertices) - 1):
                p_prev, p_curr, p_next = vertices[i - 1], vertices[i], vertices[i + 1]

                # 3a. Apex distance to lidar constraint
                corner_dist = np.linalg.norm(p_curr)
                if not (self.min_corner_dist <= corner_dist <= self.max_corner_dist):
                    continue

                arm1 = np.linalg.norm(p_prev - p_curr)
                arm2 = np.linalg.norm(p_next - p_curr)

                # 3b. Arm length constraint
                if not (self.min_arm_length <= arm1 <= self.max_arm_length):
                    continue
                if not (self.min_arm_length <= arm2 <= self.max_arm_length):
                    continue

                # 3c. Arms must be balanced (not a long wall corner)
                if max(arm1, arm2) / min(arm1, arm2) > self.max_arm_ratio:
                    continue

                v1 = (p_prev - p_curr) / arm1
                v2 = (p_next - p_curr) / arm2

                cosine_angle = np.clip(np.dot(v1, v2), -1.0, 1.0)
                angle_deg = np.degrees(np.arccos(cosine_angle))

                for target_angle in self.target_angles:
                    deviation = abs(angle_deg - target_angle)
                    if deviation <= self.angle_tolerance:
                        # Score : meilleure correspondance angulaire = score plus haut
                        score = self.angle_tolerance - deviation
                        if best_corner is None or score > best_corner[4]:
                            best_corner = (p_prev, p_curr, p_next, target_angle, score, cluster_mask)
                        break

        # 4. History update
        if best_corner is not None:
            p_prev, p_curr, p_next, angle_type, _, best_cluster_mask = best_corner
            best_cluster_scan_idx = original_indices[best_cluster_mask]

            stable_apex, bisector_away, indices_to_keep = self._fit_stable_corner(
                best_cluster_scan_idx, ranges, angles, p_curr
            )

            if stable_apex is None:
                # Fall back to approxPolyDP values
                stable_apex = p_curr
                v1 = (p_prev - p_curr) / np.linalg.norm(p_prev - p_curr)
                v2 = (p_next - p_curr) / np.linalg.norm(p_next - p_curr)
                bisector_away = v1 + v2
                bisector_away /= np.linalg.norm(bisector_away)

            raw_detection = self.compute_detection(stable_apex, bisector_away, angle_type, msg.header)
            self._register_detection(raw_detection)
        else:
            self._register_no_detection()

        # 5. Filtered scan for RViz
        self._publish_filtered_scan(msg, indices_to_keep)

    # ── Temporal filter ──────────────────────────────────────────────────────

    def _register_detection(self, raw_detection):
        px = raw_detection.pose.position.x
        py = raw_detection.pose.position.y
        # Extract yaw from 2D quaternion (only z and w are non-zero)
        raw_yaw = 2.0 * math.atan2(
            raw_detection.pose.orientation.z,
            raw_detection.pose.orientation.w,
        )

        # Check consistency with the last accepted detection
        if self._pose_history:
            last = self._pose_history[-1]

            pos_jump = math.hypot(px - last[0], py - last[1])
            yaw_jump = abs(math.atan2(
                math.sin(raw_yaw - last[2]),
                math.cos(raw_yaw - last[2]),
            ))

            if pos_jump > self.max_jump:
                # Position jump: reset history and accept the new position
                self._pose_history.clear()
                self._consecutive_detections = 0
            elif yaw_jump > self.max_yaw_jump:
                # Aberrant yaw: reject the frame without polluting the history
                self._consecutive_detections = 0
                self.get_logger().debug(
                    f"Yaw rejeté : brut={math.degrees(raw_yaw):.1f}° "
                    f"delta={math.degrees(yaw_jump):.1f}°"
                )
                return

        self._pose_history.append((px, py, raw_yaw))
        self._consecutive_detections += 1

        if self._consecutive_detections >= self.min_consecutive:
            xs = [p[0] for p in self._pose_history]
            ys = [p[1] for p in self._pose_history]
            # Circular mean of yaw to avoid ±π discontinuities
            sin_mean = float(np.mean([math.sin(p[2]) for p in self._pose_history]))
            cos_mean = float(np.mean([math.cos(p[2]) for p in self._pose_history]))
            smoothed_yaw = math.atan2(sin_mean, cos_mean)

            smoothed = PrismDetection()
            smoothed.header = raw_detection.header
            smoothed.pose.position.x = float(np.mean(xs))
            smoothed.pose.position.y = float(np.mean(ys))
            smoothed.pose.position.z = 0.0
            smoothed.pose.orientation.z = math.sin(smoothed_yaw / 2.0)
            smoothed.pose.orientation.w = math.cos(smoothed_yaw / 2.0)
            smoothed.detected_corner = raw_detection.detected_corner
            self.detection_pub.publish(smoothed)

            # Dynamic TF: lidar_frame → prism_tool
            t = TransformStamped()
            t.header = smoothed.header
            t.child_frame_id = 'prism_tool'
            t.transform.translation.x = smoothed.pose.position.x
            t.transform.translation.y = smoothed.pose.position.y
            t.transform.translation.z = 0.0
            t.transform.rotation = smoothed.pose.orientation
            self.tf_broadcaster.sendTransform(t)

            self.get_logger().debug(
                f"coin={smoothed.detected_corner}° "
                f"x={smoothed.pose.position.x:.3f} y={smoothed.pose.position.y:.3f} "
                f"yaw_brut={math.degrees(raw_yaw):.1f}° "
                f"yaw_lisse={math.degrees(smoothed_yaw):.1f}° "
                f"history={len(self._pose_history)}"
            )

    def _register_no_detection(self):
        self._consecutive_detections = 0

    # ── Pose computation from corner ────────────────────────────────────────

    def compute_detection(self, apex_xy, bisector_away, angle_type, header):
        """Build a PrismDetection from the stable apex and bisector.

        Args:
            apex_xy:       np.ndarray (2,) — apex XY position.
            bisector_away: np.ndarray (2,) — unit vector of the bisector pointing
                           into the opening of the V (toward the interior of the corner).
            angle_type:    float — corner angle in degrees (110. or 135.).
            header:        std_msgs/Header of the source scan.
        """
        theta_away = math.atan2(bisector_away[1], bisector_away[0])

        if angle_type == 110.0:
            yaw = theta_away
        else:  # 135°
            yaw = theta_away - math.pi / 2.0
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))

        det = PrismDetection()
        det.header = header
        det.pose.position.x = float(apex_xy[0])
        det.pose.position.y = float(apex_xy[1])
        det.pose.position.z = 0.0
        det.pose.orientation.z = math.sin(yaw / 2.0)
        det.pose.orientation.w = math.cos(yaw / 2.0)
        det.detected_corner = int(angle_type)
        return det

    # ── Line fitting and stable apex ─────────────────────────────────────────

    @staticmethod
    def _fit_line(pts):
        """Fit a line to a point cloud (Nx2) using SVD.

        Returns:
            (centroid, direction) where direction is a unit vector.
        """
        centroid = pts.mean(axis=0)
        _, _, vh = np.linalg.svd(pts - centroid)
        return centroid, vh[0]

    @staticmethod
    def _line_intersection(c1, d1, c2, d2):
        """Intersection of two parametric lines (point + direction).

        Solves c1 + t*d1 = c2 + s*d2 for t. Returns the point or None if parallel.
        """
        A = np.array([[d1[0], -d2[0]], [d1[1], -d2[1]]])
        if abs(np.linalg.det(A)) < 1e-8:
            return None
        t = np.linalg.solve(A, c2 - c1)[0]
        return c1 + t * d1

    def _fit_stable_corner(self, cluster_scan_indices, ranges, angles, rough_apex):
        """Refine the apex and bisector by line fitting on each arm.

        Algorithm:
          1. Sort cluster indices in scan angular order.
          2. Find the point closest to rough_apex → apex position.
          3. Collect each arm (left / right in angular order), stopping at the
             first gap > cluster_gap_threshold.
          4. Fit a line (SVD) on each arm.
          5. Compute the intersection of both lines → stable apex.
          6. Build the bisector from the directions toward each arm centroid.

        Returns:
            (stable_apex, bisector_away, kept_indices) on success,
            (None, None, kept_indices) on failure (caller falls back to rough_apex).
        """
        if len(cluster_scan_indices) == 0:
            return None, None, []

        sorted_scan_idx = np.sort(cluster_scan_indices)
        r = ranges[sorted_scan_idx]
        a = angles[sorted_scan_idx]
        xs = r * np.cos(a)
        ys = r * np.sin(a)

        # Point closest to the approxPolyDP apex
        apex_pos = int(np.argmin(np.hypot(xs - rough_apex[0], ys - rough_apex[1])))

        # Right arm (increasing angle) — stop at first large gap
        right_pos = []
        for i in range(apex_pos + 1, len(sorted_scan_idx)):
            if math.hypot(xs[i] - xs[i - 1], ys[i] - ys[i - 1]) > self.cluster_gap_threshold:
                break
            right_pos.append(i)

        # Left arm (decreasing angle) — stop at first large gap
        left_pos = []
        for i in range(apex_pos - 1, -1, -1):
            if math.hypot(xs[i] - xs[i + 1], ys[i] - ys[i + 1]) > self.cluster_gap_threshold:
                break
            left_pos.append(i)

        kept = (
            [int(sorted_scan_idx[apex_pos])]
            + [int(sorted_scan_idx[i]) for i in right_pos]
            + [int(sorted_scan_idx[i]) for i in left_pos]
        )

        # Need at least 2 points per arm to fit a line
        if len(right_pos) < 2 or len(left_pos) < 2:
            return None, None, kept

        right_local = [apex_pos] + right_pos
        left_local  = [apex_pos] + left_pos
        right_pts = np.column_stack([xs[right_local], ys[right_local]])
        left_pts  = np.column_stack([xs[left_local],  ys[left_local]])

        c_r, d_r = self._fit_line(right_pts)
        c_l, d_l = self._fit_line(left_pts)

        stable_apex = self._line_intersection(c_r, d_r, c_l, d_l)
        if stable_apex is None:
            return None, None, kept

        # Bisector: vector from apex toward each arm centroid (apex excluded).
        # SVD eigenvectors have no guaranteed sign; this approach avoids
        # bisector flips between frames.
        right_centroid = np.array([xs[right_pos].mean(), ys[right_pos].mean()])
        left_centroid  = np.array([xs[left_pos].mean(),  ys[left_pos].mean()])

        v_r = right_centroid - stable_apex
        v_l = left_centroid  - stable_apex
        v_r_norm = np.linalg.norm(v_r)
        v_l_norm = np.linalg.norm(v_l)
        if v_r_norm < 1e-8 or v_l_norm < 1e-8:
            return None, None, kept

        bisector = v_r / v_r_norm + v_l / v_l_norm
        norm = np.linalg.norm(bisector)
        if norm < 1e-8:
            return None, None, kept
        bisector /= norm

        return stable_apex, bisector, kept

    # ── Filtered scan for visualization ─────────────────────────────────────

    def _publish_filtered_scan(self, msg, indices_to_keep):
        filtered_msg = LaserScan()
        filtered_msg.header = msg.header
        filtered_msg.angle_min = msg.angle_min
        filtered_msg.angle_max = msg.angle_max
        filtered_msg.angle_increment = msg.angle_increment
        filtered_msg.time_increment = msg.time_increment
        filtered_msg.scan_time = msg.scan_time
        filtered_msg.range_min = msg.range_min
        filtered_msg.range_max = msg.range_max
        filtered_ranges = np.full(len(msg.ranges), float('inf'))
        for idx in indices_to_keep:
            filtered_ranges[idx] = msg.ranges[idx]
        filtered_msg.ranges = filtered_ranges.tolist()
        self.filtered_scan_pub.publish(filtered_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PrismDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
