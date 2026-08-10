# leeloo_msgs

Custom ROS 2 interfaces (action + service) for the Leeloo robot. No nodes — build this package first, its consumers ([leeloo_calibration](../leeloo_calibration/)) depend on it.

## Interfaces

### `action/RunCalibration` ([action/RunCalibration.action](action/RunCalibration.action))

Drives the full hand-eye calibration procedure. Consumed by `leeloo_calibration`'s `auto_calibration_server`, exposed as `/run_calibration`.

**Goal**

| Field | Type | Description |
|---|---|---|
| `num_poses` | `int32` | Number of poses to run. `0` = all poses in the file. |
| `wait_between_poses_s` | `float64` | Delay between captures. `0` falls back to the server's default. |
| `handeye_server` | `string` | Which camera to calibrate, e.g. `'kinect'`, `'oak'`. Empty → server's `default_handeye_server` param. |
| `poses_file` | `string` | Path to the poses YAML to sweep. Empty → server's `calibration_poses_file` param. |
| `camera_topic` | `string` | Used to name the output result file + for traceability. Empty → legacy `kinect_hand_eye_result.yaml` name. |
| `calibration_type` | `string` | `'eye_in_hand'` \| `'eye_on_base'`. Empty → no cross-check against the live `handeye_server` config. |

**Result**

| Field | Type | Description |
|---|---|---|
| `success` | `bool` | |
| `poses_completed` | `int32` | |
| `poses_failed` | `int32` | |
| `message` | `string` | |
| `transform` | `geometry_msgs/Transform` | Computed calibration transform. |

**Feedback**

| Field | Type | Description |
|---|---|---|
| `current_pose` | `int32` | |
| `total_poses` | `int32` | |
| `status` | `string` | `"Generating"` \| `"Executing"` \| `"Capturing"` \| `"Done"` |
| `percent_complete` | `float32` | |

### `srv/GotoPose` ([srv/GotoPose.srv](srv/GotoPose.srv))

```
int32 index
---
bool success
string message
```

Drives the arm to a previously saved calibration pose by index, then restores teach mode. Consumed by `leeloo_calibration`'s `pose_saver_node` (`/goto_pose` service).

## Build

```bash
colcon build --packages-select leeloo_msgs
source install/setup.bash
```

Build this before any package that depends on it (`ament_cmake` + `rosidl_default_generators`).
