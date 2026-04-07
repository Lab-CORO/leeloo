# object_lidar_calib

ROS 2 package for calibrating the position of a reflective prism tool relative to a 2D LiDAR. The calibration determines the tool center (TCP) in the `odom` frame by detecting two different corners of the prism from two robot positions, then publishing a static TF.

## Overview

The prism has two usable corners:
- **110°** — detected when facing the narrow side of the prism
- **135°** — detected when facing the wide side of the prism

Each corner's apex is detected from a LiDAR scan. A known geometric offset is applied to compute the tool center. Two captures from different corners are averaged to reduce measurement error.

## Nodes

### `prism_detector_node`

Processes 2D LiDAR scans to detect the prism corner in the robot's field of view.

**Pipeline:**
1. Filter scan points by range and frontal FOV (±90°)
2. Cluster points with DBSCAN
3. Approximate each cluster contour with `cv2.approxPolyDP`
4. Score candidate corners against the target angles (110° or 135°)
5. Refine the apex and bisector direction via SVD line fitting on each arm
6. Apply a temporal filter (sliding average over 5 frames, min 3 consecutive detections)

**Subscribed topics:**

| Topic | Type | Description |
|---|---|---|
| `/r100_0597/sensors/lidar2d_0/scan` | `sensor_msgs/LaserScan` | Raw 2D LiDAR scan |

**Published topics:**

| Topic | Type | Description |
|---|---|---|
| `/prism_detection` | `object_lidar_calib_msgs/PrismDetection` | Smoothed apex pose and detected corner type |
| `/filtered_scan` | `sensor_msgs/LaserScan` | Scan points kept by the detector (for visualization in RViz) |

**Published TF:**

| Child frame | Parent frame | Description |
|---|---|---|
| `prism_tool` | `lidar2d_0_laser` | Dynamic TF of the detected apex in the LiDAR frame |

---

### `Calibration_node`

Computes the tool center in `odom` from two corner captures and publishes a static TF.

**Workflow:**
1. Start both nodes.
2. Position the robot so the 110° corner is visible.
3. Call the `capture_calibration_data` service (capture 1/2).
4. Move the robot so the 135° corner is visible.
5. Call the service again (capture 2/2).
6. The static TF `prism_center` is published in `odom`.

**Subscribed topics:**

| Topic | Type | Description |
|---|---|---|
| `/prism_detection` | `object_lidar_calib_msgs/PrismDetection` | Detection output from `prism_detector_node` |
| `/r100_0597/platform/odom/filtered` | `nav_msgs/Odometry` | Robot odometry (`odom → base_link`) |
| `/r100_0597/tf` | `tf2_msgs/TFMessage` | Robot dynamic TF tree |
| `/r100_0597/tf_static` | `tf2_msgs/TFMessage` | Robot static TF tree |

**Services:**

| Service | Type | Description |
|---|---|---|
| `capture_calibration_data` | `std_srvs/Trigger` | Trigger a capture. Call twice (different corners) to complete calibration. |

**Published TF:**

| Child frame | Parent frame | Description |
|---|---|---|
| `prism_center` | `odom` | Static TF of the tool center, published after the second capture |

## Dependencies

- `rclpy`
- `object_lidar_calib_msgs`
- `tf2_ros`
- `numpy`
- `opencv-python` (`cv2`)
- `scikit-learn` (DBSCAN)

## Build

```bash
cd ros_ws/
source install/setup.bash
colcon build --packages-select object_lidar_calib
```

## Usage

```bash
# Terminal 1 — prism detector
ros2 run object_lidar_calib prism_detector_node

# Terminal 2 — calibration node
ros2 run object_lidar_calib Calibration_node

# Capture 1/2 (110° corner visible)
ros2 service call /capture_calibration_data std_srvs/srv/Trigger {}

# Move robot, then capture 2/2 (135° corner visible)
ros2 service call /capture_calibration_data std_srvs/srv/Trigger {}
```

## Geometry

The offset between the detected apex and the true tool center is `OFFSET_M = -0.085 m`, applied along the LiDAR axis corresponding to the corner type:

- 110° corner → along the LiDAR **X** axis
- 135° corner → along the LiDAR **Y** axis

The final tool center is the average of the two estimates. The published `prism_center` frame has a fixed 90° yaw rotation around Z relative to the `odom` frame.
