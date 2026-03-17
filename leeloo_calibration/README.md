# leeloo_calibration

ROS 2 package for **automated hand-eye calibration** of the Leeloo robot.
The procedure moves the robot through a set of predefined poses using [curobo_ros](https://github.com/Lab-CORO/curobo_ros), captures a calibration sample at each stop via [ros2_handeye_calibration](https://github.com/Lab-CORO/ros2_handeye_calibration), and publishes the resulting camera TF.

---

## Nodes

| Node | Role |
|---|---|
| `auto_calibration_server` | Action server that drives the full calibration procedure |
| `kinect_tf_computation_node` | Reads the calibration result and publishes the `base_link → camera_base` TF |
| `pose_saver_node` | Helper to record calibration poses manually in teach-and-repeat mode |

The action interface is defined in [`leeloo_msgs`](../leeloo_msgs/).

---

## Dependencies

| Package | Role |
|---|---|
| `curobo_ros` | Trajectory planning and execution (GPU) |
| `curobo_msgs` | `TrajectoryGeneration` + `SendTrajectory` interfaces |
| `ros2_handeye_calibration` | Hand-eye service (`/hand_eye_calibration/capture_point`) |
| `ros2_markertracker` | ArUco/AprilTag marker detection (provides the `marker` TF frame) |
| `ros2_markertracker_interfaces` | `CapturePoint` service definition |
| `azure_kinect_ros_driver` | Kinect camera driver |

---

## Installation

```bash
# From the workspace root inside the docker
cd /home/ros2_ws

# Build leeloo_msgs first (leeloo_calibration depends on it)
colcon build --packages-select leeloo_msgs
source install/setup.bash

# Build leeloo_calibration
colcon build --packages-select leeloo_calibration
source install/setup.bash
```

> **Or in one command** (colcon resolves the order automatically):
> ```bash
> colcon build --packages-select leeloo_msgs leeloo_calibration
> source install/setup.bash
> ```

---

## Usage

### Option A — Full automated calibration

Launch all required nodes with the bringup file:

```bash
ros2 launch leeloo bringup_leeloo.launch.py
```

This starts the Doosan robot, the Kinect driver, the ArUco tracker, the hand-eye calibration service, and the two calibration nodes (`kinect_tf_computation_node` + `auto_calibration_server`).

Then trigger the calibration procedure:

```bash
ros2 action send_goal /run_calibration \
    leeloo_msgs/action/RunCalibration \
    "{num_poses: 0, wait_between_poses_s: 2.0}" \
    --feedback
```

`num_poses: 0` means all poses defined in `config/calibration_poses.yaml`.

Expected feedback:
```
Feedback:
  current_pose: 3
  total_poses: 15
  status: Capturing
  percent_complete: 23.1
...
Result:
  success: True
  poses_completed: 15
  poses_failed: 0
  message: "Calibration terminée : 15/15 OK."
```

The calibration result is saved to `config/kinect_hand_eye_result.yaml` and `kinect_tf_computation_node` is reloaded automatically.

---

### Option B — Record poses manually (`pose_saver_node`)

Use this node to teach calibration poses by moving the robot by hand:

```bash
ros2 run leeloo_calibration pose_saver_node
```

The robot is automatically switched to manual mode (teach pendant). To save the current joint position:

```bash
ros2 service call /save_pose std_srvs/srv/Trigger {}
```

Poses are appended to `config/calibration_poses.yaml` in real time.
Run the automated calibration (Option A) after recording enough poses (minimum 4, recommended 15+).

---

### Cancel a running calibration

```bash
ros2 action cancel_goal /run_calibration <goal_id>
```

---

## Action Interface

### `leeloo_msgs/action/RunCalibration`

**Goal** (input):

| Field | Type | Description |
|---|---|---|
| `num_poses` | `int32` | Number of poses to execute. `0` = all poses in the YAML file |
| `wait_between_poses_s` | `float64` | Delay in seconds between captures. Default: `5.0` |

**Feedback** (progress):

| Field | Type | Description |
|---|---|---|
| `current_pose` | `int32` | Current pose index |
| `total_poses` | `int32` | Total number of poses |
| `status` | `string` | Current step (`Waiting`, `Generating`, `Executing`, `Capturing`, `Done`) |
| `percent_complete` | `float32` | Overall progress (%) |

**Result**:

| Field | Type | Description |
|---|---|---|
| `success` | `bool` | `true` if no pose failed |
| `poses_completed` | `int32` | Number of successful poses |
| `poses_failed` | `int32` | Number of skipped poses |
| `message` | `string` | Human-readable summary |
| `transform` | `geometry_msgs/Transform` | Final calibration transform (`base_link → rgb_camera_link`) |

---

## Node Parameters

### `auto_calibration_server`

| Parameter | Default | Description |
|---|---|---|
| `capture_point_service` | `/hand_eye_calibration/capture_point` | Sample capture service |
| `traj_generate_service` | `/unified_planner/generate_trajectory` | Trajectory planning service |
| `traj_execute_action` | `/unified_planner/execute_trajectory` | Trajectory execution action |
| `set_planner_service` | `/unified_planner/set_planner` | Planner type selection service |
| `calibration_poses_file` | `.../config/calibration_poses.yaml` | Poses YAML file |
| `calibration_result_file` | `.../config/kinect_hand_eye_result.yaml` | Output calibration file |
| `settle_time_s` | `2.0` | Robot settle time before each capture (seconds) |
| `reload_calibration_service` | `/kinect_tf_computation_node/reload_calibration` | Hot-reload service for TF node |
| `reset_samples_service` | `/hand_eye_calibration/reset_samples` | Reset previous samples before a new run |

### `kinect_tf_computation_node`

| Parameter | Default | Description |
|---|---|---|
| `calibration_result_file` | `.../config/kinect_hand_eye_result.yaml` | YAML file produced by `auto_calibration_server` |

Exposes service `~/reload_calibration` (`std_srvs/Trigger`) to hot-reload the calibration file without restarting.

### `pose_saver_node`

| Parameter | Default | Description |
|---|---|---|
| `robot_mode_service` | `/dsr01/system/set_robot_mode` | DSR robot mode service |
| `safety_mode_service` | `/dsr01/system/set_safety_mode` | DSR safety mode service |
| `joint_states_topic` | `/dsr01/joint_states` | Joint states topic |
| `output_file` | `.../config/calibration_poses.yaml` | Output poses file |

---


## Related Packages

- [`curobo_ros`](https://github.com/Lab-CORO/curobo_ros) — GPU trajectory planning with cuRobo
- [`curobo_msgs`](https://github.com/Lab-CORO/curobo_msgs) — ROS 2 interfaces for curobo_ros
- [`ros2_handeye_calibration`](https://github.com/Lab-CORO/ros2_handeye_calibration) — Tsai-Lenz hand-eye calibration
- [`ros2_markertracker`](https://github.com/Lab-CORO/ros2_markertracker) — ArUco/AprilTag marker detection
