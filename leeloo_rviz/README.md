# leeloo_rviz

One coherent RViz2 visualization + interaction session for the **leeloo** robot
(Doosan m1013 arm `dsr01` + Clearpath r100 mobile base + Azure Kinect), which has
no onboard UI.

This package is **launch-only**. It starts **no drivers** — it attaches to an
already-running robot bringup and just opens RViz2 (plus an optional trajectory
"ghost" robot).

## Prerequisites

The robot bringup must already be running, so that these exist:

- `/leeloo/tf` + `/leeloo/tf_static` — the **unified** TF tree (arm + base +
  cameras). RViz is remapped onto these; the bare `/tf` topics only carry
  per-subsystem transforms.
- `/leeloo/robot_description` — the combined-robot URDF used by the RobotModel.

```bash
# terminal A — robot
ros2 launch leeloo bringup_leeloo.launch.py
```

## Run

```bash
cd ~/Documents/workspaces/ros2_ws
colcon build --packages-select leeloo_rviz
source install/setup.bash

# default: robot + cameras + trajectory preview
ros2 launch leeloo_rviz leeloo_rviz.launch.py

# cuRobo collision-sphere setup workflow (no cameras)
ros2 launch leeloo_rviz leeloo_rviz.launch.py variant:=curobo
```

Launch arguments:

- `variant` (`view` | `curobo`, default `view`) — selects the shipped config
  (see "Two configs" below).
- `rviz_config` — path to a specific `.rviz` file, overriding `variant`.
- `launch_preview` (default `true`) — start the ghost/preview arm pipeline. Set
  `false` if a `/preview` pipeline is already running elsewhere (e.g. from the
  bringup) to avoid duplicate nodes.

## Two configs (why)

The reused cuRobo RViz plugins do **not** survive being combined with a full
camera view in one RViz process:

- `curobo_robot_setup/CuRoboSetupPanel` **crashes** (`failed to create drawable`)
  as soon as any camera **Image** display is also loaded — the two contend for GL
  render surfaces. They cannot coexist.
- `curobo_rviz/RvizArgsPanel` and `add_objects_panel/AddObjectsPanel` **hang**
  RViz on load (a blocking service call that never returns — their own
  `curobo_rviz/config.rviz` hangs the same way). They are not used here.

So there are two configs:

### `variant:=view` (default) — `rviz/leeloo.rviz`

Live visualization + trajectory preview. **Panels:** standard + `TrajectoryPreviewPanel`.

**Displays enabled by default (arm + perception priority):**

- Grid
- RobotModel **(real)** ← `/leeloo/robot_description`
- Trajectory preview **(ghost)** ← `/preview/robot_description`, translucent overlay
- TF (full merged tree)
- Collision spheres ← `/collision_spheres`, `/curobo_trajectory_planner/collision_spheres`
- Visualization markers ← `/visualization_markers`
- RGB camera ← `/rgb/image_raw`, Depth camera ← `/depth/image_raw`

**Present but disabled by default** (toggle on in the Displays panel):

- Arm goal marker ← `/simple_marker`
- Base teleop marker ← `/r100_0597/twist_marker_server`
- IR camera ← `/ir/image_raw`
- Base lidar ← `/r100_0597/sensors/lidar2d_0/scan`
- Fiducial markers ← `/fiducial_markers`

### `variant:=curobo` — `rviz/leeloo_curobo.rviz`

cuRobo collision-sphere setup workflow. **Panels:** standard + `CuRoboSetupPanel`.
**No camera Image displays** (they crash the panel). Displays: Grid, RobotModel
(real), TF, collision spheres, and a disabled `curobo_rviz/ArrowInteractionDisplay`
+ `/simple_marker` arm-goal marker.

## Trajectory preview ("ghost" robot)

`TrajectoryPreviewPanel` subscribes to a `trajectory_msgs/JointTrajectory` on
`/trajectory` and republishes an interpolated `sensor_msgs/JointState` on
`/trajectory/joint_states` as you scrub/play. The launch runs a `preview`-namespaced
`robot_state_publisher` (arm-only m1013 description, `frame_prefix: preview/`) driven
by that stream, grafted onto the real arm via a static TF
`dsr01/world → preview/world`. A second, translucent RobotModel renders it. The ghost
moves through the trajectory while the real arm stays put.

The ghost's `/tf[_static]` are remapped onto `/leeloo/tf[_static]` so RViz sees the
`preview/`-prefixed frames.

## Default fixed frame

`base_link` (mobile-base chassis). Change it live in **Global Options → Fixed
Frame** — e.g. `dsr01/world` to keep the arm centered, or `odom` for a
world-fixed view.

## Notes / things to confirm on the real robot

- **Trajectory joint names** — the ghost uses the dsr_description2 m1013 joints
  `joint_1..joint_6`. Confirm `/trajectory` uses the same names
  (`ros2 topic echo /trajectory --once`); adjust if the planner emits others.
- **Voxel grid** — a `/curobo_trajectory_planner/voxel_grid_sparse` display was
  left out pending its message type; add a `MarkerArray` or `PointCloud2` display
  once confirmed.
- **Arm goal arrow / `/simple_marker`** — from the standalone curobo_rviz workflow;
  disabled by default until confirmed the leeloo planner consumes it. Reference
  frame is set to `dsr01/world`.
