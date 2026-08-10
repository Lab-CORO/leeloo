# leeloo_rviz

One coherent RViz2 visualization + interaction session for the **leeloo** robot (Doosan m1013 arm `dsr01` + Clearpath r100 mobile base + Azure Kinect), which has no onboard UI.

This package is **launch-only**. It starts **no drivers** — it attaches to an already-running robot bringup and just opens RViz2 (plus an optional trajectory "ghost" robot).

## Prerequisites

The robot bringup must already be running, so that these exist:

- `/leeloo/tf` + `/leeloo/tf_static` — the **unified** TF tree (arm + base + cameras). RViz is remapped onto these; the bare `/tf` topics only carry per-subsystem transforms.
- `/leeloo/robot_description` — the combined-robot URDF used by the RobotModel.

```bash
# terminal A — robot
ros2 launch leeloo bringup_leeloo.launch.py
```

## Nodes / launch content

Nothing here is a driver — everything is either RViz2 itself or the optional "ghost" preview pipeline:

- `rviz2` (named `leeloo_rviz`), TF remapped onto `/leeloo/tf[_static]`, loaded with the config selected by `variant`/`rviz_config`.
- **Ghost preview pipeline** (`launch_preview:=true`, the default): a `preview`-namespaced `joint_state_publisher` (folds `/trajectory/joint_states` into a continuous stream) → `preview`-namespaced `robot_state_publisher` (arm-only m1013 description, `frame_prefix: preview/`) → a static `tf2_ros static_transform_publisher` grafting `preview/base_0` onto the real arm's `dsr01/base_link`. All three have `/tf[_static]` remapped onto `/leeloo/tf[_static]` too, so RViz sees the `preview/`-prefixed frames without a separate relay.

## Launch files

```bash
cd /home/ros2_ws
colcon build --packages-select leeloo_rviz
source install/setup.bash

# default: robot + cameras + trajectory preview
ros2 launch leeloo_rviz leeloo_rviz.launch.py

# cuRobo collision-sphere setup workflow (no cameras)
ros2 launch leeloo_rviz leeloo_rviz.launch.py variant:=curobo
```

| Argument | Default | Description |
|---|---|---|
| `variant` | `view` | `view` \| `curobo` — selects the shipped config (see "Two configs" below) |
| `rviz_config` | (derived from `variant`) | Path to a specific `.rviz` file, overriding `variant` |
| `launch_preview` | `true` | Start the ghost/preview arm pipeline. Set `false` if a `/preview` pipeline is already running elsewhere (e.g. from the bringup) to avoid duplicate nodes. |

## Configuration

### Two configs (why)

The reused cuRobo RViz plugins do **not** survive being combined with a full camera view in one RViz process:

- `curobo_robot_setup/CuRoboSetupPanel` **crashes** (`failed to create drawable`) as soon as any camera **Image** display is also loaded — the two contend for GL render surfaces. They cannot coexist.
- `curobo_rviz/RvizArgsPanel` and `add_objects_panel/AddObjectsPanel` **hang** RViz on load (a blocking service call that never returns — their own `curobo_rviz/config.rviz` hangs the same way). They are not used here.

So there are two configs:

#### `variant:=view` (default) — [`rviz/leeloo.rviz`](rviz/leeloo.rviz)

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

#### `variant:=curobo` — [`rviz/leeloo_curobo.rviz`](rviz/leeloo_curobo.rviz)

cuRobo collision-sphere setup workflow. **Panels:** standard + `CuRoboSetupPanel`. **No camera Image displays** (they crash the panel). Displays: Grid, RobotModel (real), TF, collision spheres, and a disabled `curobo_rviz/ArrowInteractionDisplay` + `/simple_marker` arm-goal marker.

### Trajectory preview ("ghost" robot)

`TrajectoryPreviewPanel` subscribes to a `trajectory_msgs/JointTrajectory` on `/trajectory` and republishes an interpolated `sensor_msgs/JointState` on `/trajectory/joint_states` as you scrub/play. The ghost moves through the trajectory while the real arm stays put; see [urdf/m1013_ghost.urdf](urdf/m1013_ghost.urdf) for the ghost description.

### Default fixed frame

`base_link` (mobile-base chassis). Change it live in **Global Options → Fixed Frame** — e.g. `dsr01/world` to keep the arm centered, or `odom` for a world-fixed view.

## Dependencies

RViz core (`rviz2`, `rviz_common`, `rviz_default_plugins`); reused panels `curobo_robot_setup`, `curobo_rviz`, `trajectory_preview`; ghost pipeline (`robot_state_publisher`, `joint_state_publisher`, `tf2_ros`, `xacro`, `urdf_visualizer`); camera transports (`image_transport`, `compressed_image_transport`); `interactive_markers`. See [package.xml](package.xml).

## Notes / caveats

- **Trajectory joint names** — the ghost uses the dsr_description2 m1013 joints `joint_1..joint_6`. Confirm `/trajectory` uses the same names (`ros2 topic echo /trajectory --once`); adjust if the planner emits others.
- **Voxel grid** — a `/curobo_trajectory_planner/voxel_grid_sparse` display was left out pending its message type; add a `MarkerArray` or `PointCloud2` display once confirmed.
- **Arm goal arrow / `/simple_marker`** — from the standalone curobo_rviz workflow; disabled by default until confirmed the leeloo planner consumes it. Reference frame is set to `dsr01/world`.
