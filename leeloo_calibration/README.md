# leeloo_calibration

ROS 2 package for **hand-eye calibration** of Leeloo's cameras (Kinect eye-on-base, OAK-D SR eye-in-hand), both automated and manual/interactive. The automated procedure moves the arm through a set of predefined poses via [curobo_ros](https://github.com/Lab-CORO/curobo_ros), captures a sample at each stop via [ros2_handeye_calibration](https://github.com/Lab-CORO/ros2_handeye_calibration) (`easy_handeye2`), and computes/saves the resulting camera transform.

`kinect_tf_computation_node` does **not** calibrate anything — it replays a saved calibration result as a live TF, runs at all times, and lives in [leeloo/launch/tf.launch.py](../leeloo/launch/tf.launch.py), not in this package's launch file.

---

## Nodes

| Node | Role |
|---|---|
| `auto_calibration_server` | Action server (`/run_calibration`) that drives the full automated calibration procedure for one camera at a time |
| `kinect_tf_computation_node` | Reads a saved calibration result and publishes the `<robot_base_frame> → camera_base` static TF. Runs permanently (see above); exposes `~/reload_calibration` and `~/toggle_publication` |
| `pose_generator_node` | Interactive: generates ~15 candidate poses around a user-taught reference pose, drives the arm to each, and lets the user confirm/skip based on marker visibility |
| `pose_saver_node` | Interactive: puts the arm in teach mode, saves the current joint pose on request. Also exposes `/goto_pose` to drive back to a saved pose, and `/info_calibration` for a workspace-coverage report |
| `manual_tf_calibration_node` | Lets a user tune the `base_link → camera_base` transform live via 6 float topics, for eyeballing/fine-tuning in RViz without running a full calibration |

The action interface is defined in [`leeloo_msgs`](../leeloo_msgs/).

---

## Dependencies

| Package | Role |
|---|---|
| `curobo_ros` | Trajectory planning and execution (GPU) |
| `curobo_msgs` | `TrajectoryGeneration` + `SendTrajectory` interfaces |
| `ros2_handeye_calibration` (`easy_handeye2`) | Hand-eye service (`take_sample`, `compute_calibration`, …) |
| `ros2_markertracker` | ArUco/AprilTag marker detection (provides the `marker` TF frame) |
| `ros2_markertracker_interfaces` | `FiducialMarkerArray` message |
| `leeloo_msgs` | `RunCalibration` action, `GotoPose` service |
| `dsr_msgs2` | Doosan `SetRobotMode` / `SetSafetyMode` services (teach mode) |

---

## Installation

```bash
cd /home/ros2_ws
colcon build --packages-select leeloo_msgs leeloo_calibration   # colcon resolves the build order
source install/setup.bash
```

---

## Usage

### Launch

```bash
ros2 launch leeloo_calibration calibration.launch.py
```

Runnable standalone (applies its own `/tf` → `/leeloo/tf` remap) or, more commonly, included from [bringup_leeloo.launch.py](../leeloo/launch/bringup_leeloo.launch.py). Starts, per enabled camera: a dedicated calibration-marker tracker, an `easy_handeye2` `handeye_server` (remapped to `/easy_handeye2_<camera>/calibration/*` so multiple cameras can calibrate without service collisions), and `auto_calibration_server`.

| Argument | Default | Description |
|---|---|---|
| `enabled_cameras` | `kinect,oak` | Only starts a camera's marker tracker / `handeye_server` if that camera is in this list |
| `enable_calibration` | `true` | Set `false` in production: without a calibration marker in view, `handeye_server` never creates its services and logs an ERROR every ~10 s, and `auto_calibration_server` eventually times out waiting for it. The application marker tracker keeps running regardless — it isn't part of calibration. |
| `sim` | `false` | Simulation provides no RGB stream, so the whole marker-detection chain is skipped when `sim:=true` |

### Option A — Automated calibration

```bash
ros2 action send_goal /run_calibration \
    leeloo_msgs/action/RunCalibration \
    "{num_poses: 0, wait_between_poses_s: 2.0, handeye_server: 'kinect', calibration_type: 'eye_on_base'}" \
    --feedback
```

`num_poses: 0` means all poses in the resolved poses file. `handeye_server` selects the camera (`'kinect'` or `'oak'` with the shipped launch file); empty defaults to the server's `default_handeye_server` param (`kinect`).

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

Result is saved to `config/kinect_hand_eye_result.yaml` (or `<camera_topic>_hand_eye_result.yaml` if `camera_topic` was set in the goal) and, only for the `default_handeye_server` camera, `kinect_tf_computation_node` is reloaded automatically. For any other camera the result is saved but must be applied manually.

Cancel:
```bash
ros2 action cancel_goal /run_calibration <goal_id>
```

### Option B — Record poses manually (`pose_saver_node`)

```bash
ros2 run leeloo_calibration pose_saver_node
```

Puts the arm in manual/teach mode automatically. Move it by hand, then:

```bash
ros2 service call /save_pose std_srvs/srv/Trigger {}
```

Poses accumulate in `config/calibration_poses.yaml`. Duplicate poses (joint-space distance < 0.5 rad from an existing one) are rejected. `/info_calibration` (`std_srvs/Trigger`) reports workspace coverage (j1×j2 zone coverage, elbow-up/down balance) against a recommended minimum of 15 poses. `/goto_pose` (`leeloo_msgs/GotoPose`, field `index`) drives back to a saved pose for re-inspection, then restores teach mode.

### Option C — Interactive pose generation (`pose_generator_node`)

```bash
ros2 run leeloo_calibration pose_generator_node
```

Alternative to Option B: instead of teaching every pose by hand, teach one reference pose (marker centered in camera), then let the node propose ~15 candidates (wrist-only variations first, then arm-position variations) around it.

```bash
ros2 service call /set_reference_pose std_srvs/srv/Trigger {}
# for each candidate the arm drives to:
ros2 service call /confirm_pose std_srvs/srv/Trigger {}   # saves current joint pose if valid
# or, if unreachable / marker not visible:
ros2 service call /skip_pose std_srvs/srv/Trigger {}
```

Writes the same `config/calibration_poses.yaml` format as Option B.

### Option D — Manual TF tuning (`manual_tf_calibration_node`)

```bash
ros2 run leeloo_calibration manual_tf_calibration_node
```

For eyeballing/fine-tuning `base_link → camera_base` live in RViz rather than running a full calibration. Publishes the TF at 50 Hz from 6 `std_msgs/Float64` topics:

```bash
ros2 topic pub /calibration_z std_msgs/msg/Float64 "{data: 0.65}" -1
ros2 topic pub /calibration_theta_z std_msgs/msg/Float64 "{data: 90.0}" -1
ros2 service call /save_calibration std_srvs/srv/Trigger {}
```

**Do not run this at the same time as `kinect_tf_computation_node`** — both publish `camera_base` and would conflict. On startup this node calls `kinect_tf_computation_node`'s `~/toggle_publication` service to disable it automatically.

### Offline recalibration from saved samples

If `auto_calibration_server` was run with `enable_debug_capture: true`, every captured sample (robot + tracking transforms, marker ambiguity/reprojection error) is archived to `config/calibration_debug_samples.yaml`. [scripts/recalibrate_from_samples.py](scripts/recalibrate_from_samples.py) recomputes the hand-eye transform offline from that file, filtering by ambiguity ratio and choosing the OpenCV method — no ROS dependency, just `numpy`/`cv2`/`transforms3d`/`yaml`:

```bash
python3 scripts/recalibrate_from_samples.py --input config/calibration_debug_samples.yaml \
    --threshold 2.0 --method Tsai-Lenz --output config/kinect_hand_eye_result.yaml
```

---

## Action interface

See [`leeloo_msgs/README.md`](../leeloo_msgs/README.md#actionruncalibration-actionruncalibrationaction) for the full `RunCalibration` goal/result/feedback field reference, and [`GotoPose`](../leeloo_msgs/README.md#srvgotopose-srvgotoposesrv) for the service `pose_saver_node`'s `/goto_pose` uses.

---

## Configuration files

| File | Written by | Consumed by |
|---|---|---|
| `config/calibration_poses.yaml` | `pose_saver_node`, `pose_generator_node` | `auto_calibration_server` |
| `config/kinect_hand_eye_result.yaml` | `auto_calibration_server`, `manual_tf_calibration_node`, `recalibrate_from_samples.py` | `kinect_tf_computation_node` |
| `config/oak_calibration_poses.yaml` | (OAK-specific poses set) | `auto_calibration_server` when `handeye_server: 'oak'` |
| `config/oak_stereo_image_raw_hand_eye_result.yaml` | OAK calibration run | Applied manually — see the launch section above |
| `config/calibration_debug_samples.yaml` | `auto_calibration_server` (`enable_debug_capture: true`) | `recalibrate_from_samples.py` |

---

## Related packages

- [`curobo_ros`](https://github.com/Lab-CORO/curobo_ros) — GPU trajectory planning with cuRobo
- [`curobo_msgs`](https://github.com/Lab-CORO/curobo_msgs) — ROS 2 interfaces for curobo_ros
- [`ros2_handeye_calibration`](https://github.com/Lab-CORO/ros2_handeye_calibration) — Tsai-Lenz hand-eye calibration (`easy_handeye2`)
- [`ros2_markertracker`](https://github.com/Lab-CORO/ros2_markertracker) — ArUco/AprilTag marker detection
