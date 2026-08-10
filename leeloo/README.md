# leeloo

Core package: the execution layer between cuRobo's planner output and the real (or simulated) hardware, plus the robot's URDF, TF setup, and top-level launch files. This is the package referenced as `leeloo` throughout the rest of this repo — see the [root README](../README.md) for the system-level picture.

---

## Nodes

- **`execute_trajectory`** ([src/execute_trajectory.cpp](src/execute_trajectory.cpp)) — Consumes a `trajectory_msgs/JointTrajectory` from cuRobo and streams it to the Doosan driver as `dsr_msgs2/SpeedjRtStream`, one point per fixed wall-clock tick (`command_period`, default 0.03 s). Rejects an incoming trajectory whose `time_from_start` spacing doesn't match `command_period` (must stay in sync with cuRobo's `interpolation_dt`). Can optionally stream acceleration alongside velocity (`send_acceleration` param). Service `~/enable_streaming` (`std_srvs/SetBool`) pauses/resumes streaming, e.g. while a direct Doosan API call (compliance control) needs the channel.
  - sub `/leeloo/execute_trajectory` (`trajectory_msgs/JointTrajectory`)
  - pub `/dsr01/dsr_controller2/speedj_rt_stream` (`dsr_msgs2/SpeedjRtStream`), `/leeloo/trajectory_state` (`std_msgs/Float32`)
- **`ridgeback_control`** ([src/ridgeback_control.cpp](src/ridgeback_control.cpp)) — Relays `/cmd_vel` (`geometry_msgs/Twist`) to the base as a timestamped `geometry_msgs/TwistStamped` on `/r100_0597/joy_teleop/cmd_vel`, published continuously at 10 Hz (last received command, or zero).
- **`voxel_map_publisher`** ([src/voxel_map_publisher.cpp](src/voxel_map_publisher.cpp)) — Subscribes to cuRobo's sparse voxel grid (`/curobo_trajectory_planner/voxel_grid_sparse`) and republishes it as a `visualization_msgs/MarkerArray` on `voxel_map` for RViz, with reconstruction/latency logging. Optional legacy service-polling path (`use_service` param) for RTT comparison against the topic path.
- **`voxel_dataset_recorder.py`** ([scripts/voxel_dataset_recorder.py](scripts/voxel_dataset_recorder.py)) — Records the same sparse voxel topic into an HDF5 dataset (one file per session, one dense `uint8` grid per frame), for offline use (e.g. training data). Deduplicates identical consecutive frames by default.

> **[src/grasp.cpp](src/grasp.cpp) is not currently built or installed** — see the root README's [Known discrepancies](../README.md#known-discrepancies). It is actually a Python pick-and-place state machine (marker-tracked grasp with a compliant approach), not C++.

## Launch files

- **[bringup_leeloo.launch.py](launch/bringup_leeloo.launch.py)** — Top-level orchestrator. Declares the shared TF remap arguments and `sim`, then includes the four files below plus `leeloo_calibration/launch/calibration.launch.py`. This is the file most users launch directly.
- **[cameras.launch.py](launch/cameras.launch.py)** — Starts the Azure Kinect and/or OAK-D SR drivers, selected via `enabled_cameras`. Tolerant to a missing driver package (logs a warning and skips instead of failing the whole bringup) — useful since `sim:=true` never needs either driver.
- **[tf.launch.py](launch/tf.launch.py)** — Relays the Ridgeback's and the global `/tf[_static]` onto `/leeloo/tf[_static]`, runs the structure `robot_state_publisher`, `kinect_tf_computation_node` (replays the saved hand-eye calibration as a static TF — it is not a calibration node itself), and the combined-robot `robot_state_publisher` used for RViz.
- **[control.launch.py](launch/control.launch.py)** — Starts the Doosan bringup (`dsr_bringup2`), the Robotiq gripper's `ros2_control`, cuRobo's `curobo_trajectory_planner` (MPC) and `robot_segmentation`, and this package's `execute_trajectory` + `ridgeback_control`. Set `CUROBO_DEBUG=gdb` or `CUROBO_DEBUG=memcheck` before launch to wrap the planner node for crash debugging.

All four accept `sim` and the `tf_remap`/`tf_static_remap` pair; see [root README → Running Leeloo](../README.md#running-leeloo) for the full argument table.

## URDF & meshes

- **[urdf/leeloo.urdf](urdf/leeloo.urdf)** — Combined robot description (Ridgeback + structure + Doosan M1013 + Robotiq gripper). Used by the visualization `robot_state_publisher`, by cuRobo's kinematics/collision model, and as the base description patched in-memory by `leeloo_gazebo`.
- **[urdf/leeloo_structure.urdf](urdf/leeloo_structure.urdf)** — Just the fixed-joint bridge `default_mount → structure_base → … → robot_mount → dsr01/world`, published as static TF by `tf.launch.py`.
- [meshes/](meshes) — Visual and collision meshes for both URDFs.

## Configuration

| File | Role |
|---|---|
| [config/cameras.yaml](config/cameras.yaml) | Declares each depth camera (topic, frame, `frame_rate_hz`) for cuRobo's obstacle mapper. `frame_rate_hz` must track the actual driver rate — it drives the TSDF's decay-per-`integrate()`-call math. |
| [config/leeloo_curobo.yaml](config/leeloo_curobo.yaml) | cuRobo robot config: kinematics (`base_link`, `urdf_path`), collision spheres per link. |
| [config/leeloo_world.yaml](config/leeloo_world.yaml) | Static obstacles (floor, camera pole, …) seeded into cuRobo's ESDF. |
| [config/oak_sr_depth.yaml](config/oak_sr_depth.yaml) | OAK-D SR on-device stereo filter parameters (fps, range thresholds, speckle filter). |
| [config/profile.xml](config/profile.xml) | — |
| [config/config.yaml](config/config.yaml) | Currently empty. |

## Dependencies

`rclcpp`, `dsr_msgs2`, `curobo_msgs`, `std_srvs`, `trajectory_msgs`, `visualization_msgs`, `geometry_msgs` (build); `leeloo_calibration`, `robot_state_publisher`, `topic_tools`, `robotiq_description` (runtime, see [package.xml](package.xml)).

## Notes / caveats

- The TF remap mechanism (`tf_remap`/`tf_static_remap`) is described in detail in the [root README](../README.md#the-unified-leeloo-tf-tree) — this package's launch files are where it's implemented (`SetRemap` + `GroupAction`).
- `execute_trajectory`'s dt barrier means a mismatched `command_period` (here) vs. `interpolation_dt` (cuRobo) silently drops every incoming trajectory rather than executing it at the wrong rate — check `~/enable_streaming` and the node's logs first if the arm isn't moving.
