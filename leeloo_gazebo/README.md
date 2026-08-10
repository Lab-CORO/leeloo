# leeloo_gazebo

Gazebo Sim (Harmonic / gz-sim 8, ROS 2 Jazzy) simulation of **Leeloo** — Clearpath Ridgeback + Doosan M1013 + Robotiq 2F-85 + depth cameras — built from [`leeloo/urdf/leeloo.urdf`](../leeloo/urdf/leeloo.urdf).

This package is **not** a second software stack — it is a **hardware substitute**. Every topic it exposes carries the name and type of its real-hardware equivalent, so `curobo_ros`, `robot_segmentation`, `execute_trajectory`, `ridgeback_control` and `grasp` all run **unmodified** against the simulation; only the *producers* change. The [`leeloo`](../leeloo/) package is never modified by this file: `leeloo.urdf` is patched **in memory** at launch time (see [XML surgery](#xml-surgery-on-leelooudf) below).

---

## Quick start

### Full stack (the normal case)

```bash
cd /home/ros2_ws
colcon build --packages-select leeloo_msgs leeloo leeloo_calibration leeloo_gazebo
source install/setup.bash

ros2 launch leeloo bringup_leeloo.launch.py sim:=true enable_calibration:=false
```

`sim:=true` (default `false`) swaps hardware for this package; the rest of the stack — cuRobo, segmentation, TF, execution — is unchanged.

### Simulated hardware alone (isolated debugging)

```bash
ros2 launch leeloo_gazebo leeloo_gazebo.launch.py
```

In `standalone` mode (the default), `leeloo_gazebo` also runs `leeloo`'s real `ridgeback_control` node, so `/cmd_vel` works end-to-end without the full bringup.

### MPC timing tests

```bash
ros2 launch leeloo bringup_leeloo.launch.py sim:=true enable_calibration:=false \
  enabled_cameras:=none
```

Cameras are the dominant cost in the scene. This is a **mandatory precaution**, not an optimization — see [Timing fidelity](#timing-fidelity--read-before-drawing-mpc-conclusions).

> `enabled_cameras:=''` is rejected by launch (`malformed launch argument`). Use `none`.

---

## What gets substituted

```
                     sim:=true                              sim:=false (default)
─────────────────────────────────────────────────────────────────────────────────
 ARM     curobo_ros (JointSpeedStrategy, planner_type: mpc)          ← identical
           └→ /leeloo/execute_trajectory      (JointTrajectory)
         leeloo/execute_trajectory                                   ← identical
           └→ /dsr01/dsr_controller2/speedj_rt_stream  (SpeedjRtStream, deg/s)
                        │                                       │
                   dsr_sim_bridge                          dsr_controller2
                   → arm_velocity_controller (velocity)     → DRFL speedj_rt()
                   → gz_ros2_control                        → hardware
         feedback  /joint_states → dsr_sim_bridge           Doosan driver
                   → /dsr01/joint_states                     → /dsr01/joint_states

 BASE    /cmd_vel → leeloo/ridgeback_control                         ← identical
           └→ /r100_0597/joy_teleop/cmd_vel   (TwistStamped)
                        │                                       │
                  base_sim_bridge                        Clearpath firmware
                  → gz VelocityControl                   (wheel mixing + watchdog)
         odometry   /r100_0597/odom, /r100_0597/tf        (same topics)

 GRIPPER  grasp → /robotiq_gripper_controller/gripper_cmd (ParallelGripperCommand)
                  same controller name AND type on both sides

 CAMERAS  gz depth_camera sensors                          k4a / depthai drivers
           → /depth_to_rgb/{image_raw,camera_info}
           → /oak/stereo/{image_raw,camera_info}
                        └────────── identical ──────────┘
                  robot_segmentation → /masked_depth_image{,_oak_wrist}
                    → cuRobo voxel map / ESDF
```

`sim:=true` disconnects: `dsr_bringup2`, the real gripper's `ros2_control_node`, both camera drivers, `kinect_tf_computation_node`, `leeloo_viz_state_publisher` (replaced by this package's `robot_state_publisher`, which publishes a *superset* description) and the marker-detection chain.

---

## Nodes

| Node | Role |
|---|---|
| [`dsr_sim_bridge`](leeloo_gazebo/dsr_sim_bridge.py) | Stands in for `dsr_controller2`. Consumes `/dsr01/dsr_controller2/speedj_rt_stream`, converts `vel` **deg/s → rad/s** onto `arm_velocity_controller`, and republishes the arm's 6 joints on `/dsr01/joint_states` (same as the real driver). Watchdog: zero velocity after ~3× the expected command period with no new command. |
| [`base_sim_bridge`](leeloo_gazebo/base_sim_bridge.py) | Stands in for the Clearpath firmware. Consumes `ridgeback_control`'s real output (`TwistStamped`), clamps to `max_linear_speed`/`max_angular_speed`, republishes as `Twist` for the base's Gazebo plugin. Watchdog: 0.25 s. |

Neither watchdog is cosmetic: without it, Gazebo's velocity/control plugins hold the last command **indefinitely**, while the real firmware and DRCF stop on their own — a simulated robot would keep moving after its commander dies.

## `ros2_control` controllers

Defined in [`config/leeloo_controllers.yaml`](config/leeloo_controllers.yaml), served by `gz_ros2_control/GazeboSimSystem` at 100 Hz.

| Controller | Type | State |
|---|---|---|
| `joint_state_broadcaster` | `joint_state_broadcaster/JointStateBroadcaster` | active |
| `arm_velocity_controller` | `forward_command_controller/ForwardCommandController` (`velocity`) | **active** — cuRobo/MPC production path |
| `arm_controller` | `joint_trajectory_controller/JointTrajectoryController` (`position`) | **inactive** — ad-hoc tests without cuRobo |
| `robotiq_gripper_controller` | `parallel_gripper_action_controller/GripperActionController` | active |

`arm_velocity_controller` and `arm_controller` claim the same joints: **only one can be active**. To switch to position mode:

```bash
ros2 control switch_controllers --deactivate arm_velocity_controller \
                                --activate arm_controller
```

**The base is not in `ros2_control`.** It is driven by `gz::sim::systems::VelocityControl` (twist applied directly to the model body) paired with `gz::sim::systems::OdometryPublisher` (odometry read back from the model's true physics pose), both declared in the robot description. This replaced an earlier `gz::sim::systems::MecanumDrive` plugin — see [Base drive: why not MecanumDrive](#base-drive-why-not-mecanumdrive) below, since it's a real, verified pitfall rather than a stylistic choice.

---

## Launch arguments

| Argument | Default | Description |
|---|---|---|
| `world` | `worlds/leeloo_world.sdf` | SDF world file |
| `spawn_z` | `0.03` | Spawn height. Resting height is 0.0259 m (wheel radius 0.0759 − z-offset 0.05); the margin avoids both a fall and initial interpenetration. |
| `standalone` | `true` | Also runs `ridgeback_control` so `/cmd_vel` works alone. Set to `false` by the bringup, which already runs it. |
| `enabled_cameras` | `kinect,oak` | Same convention as `cameras.launch.py`. `none` for no cameras. |
| `use_sim_time` | `true` | See the timing-fidelity warning below. |
| `tf_remap` / `tf_static_remap` | `/leeloo/tf` / `/leeloo/tf_static` | Aligned with the real robot |
| `kinect_xyz` / `kinect_rpy` | `0 0 0.6` / `0 0.1 0.449` | **Nominal** Kinect pose on `camera_mount` — a placeholder for the hand-eye calibration result, not a measurement. Aims roughly at (0.9, 0, 0.9) in `base_link`. |
| `oak_xyz` / `oak_rpy` | same values as `cameras.launch.py` | Eye-in-hand offset on `dsr01/link_6` |

---

## Timing fidelity — read before drawing MPC conclusions

**The entire execution chain is paced on the wall clock**, not simulated time: cuRobo's MPC servo loop uses `time.time()`, `joint_speed_strategy` uses `time.monotonic()`, and `execute_trajectory.cpp` uses a `create_wall_timer`. `use_sim_time:=true` changes **none** of that — it only keeps TF/image timestamps consistent with `/clock` (without it, every TF lookup fails, since sim time starts near 0 against a wall time of ~1.7e9).

Consequence: timing fidelity requires **RTF ≈ 1**. Measured on a Jetson (aarch64/Tegra):

| Configuration | RTF |
|---|---|
| 2 cameras 1280×720, no cuRobo | **1.00** |
| 2 cameras + cuRobo idle | 0.80 – 0.88 |
| 2 cameras + MPC executing | **0.50 – 1.03**, drops to ~0.5 during solves |

**Functional** validation is solid, but MPC latency/convergence numbers **do not transfer to the real robot**. For those measurements: launch with `enabled_cameras:=none`, or use the x86 Docker image (`curobo_ros/docker/start_docker_x86.sh`).

> Camera resolution is not a free lever: 1280×720 is imposed by `mapper_image_width`/`mapper_image_height` in [`leeloo/launch/control.launch.py`](../leeloo/launch/control.launch.py).

### Startup is slow, and it looks like a hang

Gazebo only creates the root `/clock` topic after noticing no publisher exists yet (`"Found no publishers on /clock, adding root clock topic"`), and `ros_gz_bridge` then has to reconnect. For **~20–30 s**, the ROS `/clock` carries nothing, so every `use_sim_time` node stays at t=0: `/joint_states` is stamped `sec=0`, `robot_state_publisher` publishes no dynamic TF, and `controller_manager` logs a few hundred "No clock received, using time argument instead!" warnings. It all recovers on its own. **Do not debug TF or clock issues before ~60 s of uptime** — measure again first.

---

## XML surgery on `leeloo.urdf`

`leeloo.urdf` is a flat, already-xacro-expanded file with no macro to parameterize. It ships a `<ros2_control>` block for the **real** Robotiq driver (`robotiq_driver/RobotiqGripperHardwareInterface`, `COM_port /dev/ttyUSB0`), which cannot run in simulation, and it has **no camera links at all** (the Kinect's pose comes from hand-eye calibration at runtime; the OAK-D has its own `robot_state_publisher` in the depthai driver).

`leeloo_gazebo.launch.py` therefore builds the description **in memory**:

1. strips the real-hardware `<ros2_control>` blocks;
2. appends sim `<ros2_control>` blocks (`gz_ros2_control/GazeboSimSystem`) — arm with `position` **and** `velocity` interfaces, gripper with its 6 joints;
3. appends the `<gazebo>` plugin tags (`gz_ros2_control`, `VelocityControl`, `OdometryPublisher`);
4. injects both cameras: a *body* link + an *optical* frame + a `depth_camera` sensor;
5. damps the suspension pivot `front_rocker` (declared `effort="0"` with no `<dynamics>`: it oscillates freely in Gazebo otherwise).

Files under `leeloo/urdf/` are never touched.

### Camera frames

Each camera gets **two** links: a *body* link (ROS convention, +X forward) carrying the sensor — gz renders a camera along its link's +X — and a child *optical* link (OpenCV convention, +Z forward) carrying the name `cameras.yaml` expects:

- `camera_mount` → `kinect_camera_body` → **`rgb_camera_link`**
- `dsr01/link_6` → `oak_camera_body` → **`oak_left_camera_optical_frame`**

These poses stay in the OpenCV optical convention: **do not** convert them to REP-103 in this pipeline.

---

## Verification

Check for ghost processes first — see [Troubleshooting](#troubleshooting); they silently invalidate every result below.

```bash
# 1. Controllers
ros2 control list_controllers
#   arm_velocity_controller … active   |  arm_controller … inactive
#   robotiq_gripper_controller … active |  joint_state_broadcaster … active

# 2. No missing mesh (watch GZ_SIM_RESOURCE_PATH)
#    → 0 occurrences of "Failed to load mesh" in the log

# 3. RTF
gz topic -e -t /world/leeloo_world/stats -n 3 | grep real_time_factor

# 4. Gripper mimic — all SIX joints must move, not just the left knuckle
ros2 action send_goal /robotiq_gripper_controller/gripper_cmd \
  control_msgs/action/ParallelGripperCommand \
  "{command: {name: ['robotiq_85_left_knuckle_joint'], position: [0.01]}}"
ros2 topic echo /joint_states --once

# 5. Holonomic base + watchdog
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {y: 0.3}}"
ros2 topic echo /r100_0597/odom     # y increases, x stays ~0
#   then Ctrl-C: the base must STOP (watchdog)

# 6. Cameras → ESDF
ros2 topic hz /depth_to_rgb/image_raw            # ~5 Hz
ros2 topic echo /depth_to_rgb/camera_info --once # must arrive (see pitfall below)
ros2 topic hz /curobo_trajectory_planner/voxel_grid_sparse
```

### Commanding the arm via cuRobo

Publishing a `Pose` on `/curobo_trajectory_planner/mpc_goal` is **not enough**: the callback only stores the target for the servo loop, and that loop **only runs during the** `/curobo_trajectory_planner/execute_trajectory` **action** (`curobo_msgs/action/SendTrajectory`). Send the action goal with `target_pose` set. Also: the `fk`/`ik` services return empty until `warmup_fk`/`warmup_ik` has been called.

Results from validation (full bringup, `sim:=true`): cuRobo publishes on `/leeloo/execute_trajectory` at ~2.2 Hz, `execute_trajectory` streams `speedj_rt_stream` at **33.3 Hz** (= 1/`command_period` = 1/0.03) with **0** dt-barrier rejections, the arm tracks, and the ESDF fed by the simulated cameras reaches ~126,600 occupied voxels.

---

## Assumed limitations

- **`SpeedjRtStream.acc` and `.time` are ignored.** On hardware the DRCF uses them to ramp between velocity setpoints; `gz_ros2_control` applies velocity directly. Simulated motion is therefore a bit more aggressive at MPC batch boundaries. Upstream acceleration clamps (cuRobo's `max_joint_accel_dps`, `execute_trajectory`'s `min/max_acceleration_dps2`) still apply — nothing illegal is hidden, only the DRCF's own smoothing is not reproduced.
- **No Doosan alarms**, no error codes, no safety stop.
- **The base drive (`VelocityControl`) is kinematic-abstraction only**: the twist is applied to the model directly. Real (and equally real) cost: the wheel joints no longer spin (purely cosmetic — nothing drives them now), and the base no longer interacts with the ground through friction — it will not slip, and ignores slopes/surface properties.
- **Grasp contacts are not very faithful.** Even with corrected mimic joints, dartsim + the Robotiq's meshed collisions don't produce a convincing grasp. The sim validates `grasp`'s *logic* (sequencing, masking, gripper action), not the grasp itself.
- **No simulated RGB stream** → hand-eye calibration and marker detection are out of scope (the chain is automatically disabled under `sim:=true`).
- **Slightly optimistic depth coverage** for the Kinect: the simulated sensor renders valid depth over the whole color FOV, while the real depth FOV (NFOV_UNBINNED, 75×65°) is narrower.
- **Nominal Kinect pose**, meant to be adjusted: it stands in for the hand-eye calibration result.

---

## Troubleshooting

### Base drive: why not MecanumDrive

An earlier version of this launch file used `gz::sim::systems::MecanumDrive` and described it as kinematic — it is not. `MecanumDrive` converts the body twist into four **wheel-joint velocities** and relies on wheel-ground contact to turn those into body motion. `leeloo.urdf`'s wheels are plain cylinder collisions with a single revolute axis and isotropic friction, no mecanum roller geometry, so that contact can only ever produce longitudinal force — ruling out lateral motion regardless of the commanded twist. Measured on this install, reading the model's *true* physics pose (not the plugin's own odometry): commanding `linear.y = 0.2` under `MecanumDrive` moved the base **+1.391 m forward and 0.000 m sideways**. The bug was invisible because `MecanumDrive` derives its odometry by integrating the same kinematic model used to compute the wheel speeds, so `/odometry` and the `odom→base_link` TF confidently reported the strafe that never physically happened.

`VelocityControl` (paired with `OdometryPublisher` for odometry read back from the true physics pose) applies the twist to the model directly — the abstraction the real Clearpath firmware actually provides. This is documented here in detail specifically because it's an easy regression to reintroduce: don't switch the base plugin back to `MecanumDrive` without re-reading this section.

### Gripper mimic joints don't follow

They are applied by **`ros2_control`**, not by Gazebo. Verified on this install (gz-sim 8.11 / gz-physics 7.6 / sdformat 14.9): URDF→SDF conversion does emit `<axis><mimic>` correctly, and gz-sim does request `gz::physics::SetMimicConstraintFeature` — but only the **bullet-featherstone** engine implements it, and the default engine is **dartsim**, which does not. The constraint is silently dropped.

Hence the rule: **every** mimic joint must appear in the `<ros2_control>` block, **without** a `<command_interface>` (`ros2_control` refuses a command interface on an active mimic joint), with the command on the leading joint only. Do **not** switch the world to bullet-featherstone for this — it would change the dynamics of the whole robot, `gz_ros2_control` and the base drive included.

### A gz topic looks connected but nothing arrives

Two silent pitfalls, neither producing any error message:

1. **`VelocityControl`'s `<topic>` (and `OdometryPublisher`'s `<odom_topic>`/`<tf_topic>`) are not model-scoped.** A relative name (`cmd_vel`) becomes the **global** gz topic `/cmd_vel`, not `/model/leeloo/cmd_vel` the way the `DiffDrive` plugin's own default would.
2. **`camera_info`: gz *replaces* the last segment of `<topic>`, it doesn't append to it.** `<topic>kinect/depth</topic>` publishes the image on `/kinect/depth` and the info on **`/kinect/camera_info`** — *not* `/kinect/depth/camera_info`. Bridging the concatenated name creates a subscriber with no publisher: `camera_info` never arrives, cuRobo cannot unproject depth, and **the ESDF stays empty with no error anywhere**.

Always diagnose with `gz topic -i -t <name>` and read the publisher/subscriber split — a topic listed by `gz topic -l` may exist only because something **subscribes** to it.

Note: `gz_frame_id` is the tag gz-sensors (8.2) actually reads to stamp `frame_id`; `optical_frame_id` is only an sdformat schema element in this version and nothing consumes it at runtime. Both are set, and sdformat warns that `gz_frame_id` is "not defined in SDF" — that pass-through is exactly what makes it work.

### All meshes fail to load

URDF→SDF conversion rewrites every `package://<pkg>/…` URI to `model://<pkg>/…` **verbatim**, without ament resolution. Gazebo then looks for a `<pkg>` subdirectory in `GZ_SIM_RESOURCE_PATH`. Neither `leeloo` nor `robotiq_description` export `<gazebo_ros gazebo_model_path>` (they aren't Gazebo packages). The launch file handles this itself via `SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", …)`. **Any new mesh-carrying package added to the scene must be added there too.**

### "Controller already loaded" / a relaunch behaves strangely

Processes from a previous session survived. `pkill -f` is **not** reliable here — it often reports success while leaving the tree alive, and the relaunch then collides with an orphaned `controller_manager` on a non-namespaced ROS graph.

```bash
ps -eo pid,cmd | grep -iE "gz sim|ros_gz|parameter_bridge|robot_state_publisher|\
controller_manager|dsr_sim|base_sim|ridgeback|curobo|markertracker|relay|spawner|\
execute_trajectory|robot_segmentation" | grep -v grep
# then kill -9 the listed PIDs explicitly, and re-check the grep is empty
```

The pattern must cover **every** node from the launch, not just the obvious ones. Real case: the pattern omitted `execute_trajectory`; that node has `respawn=True`, and a survivor doubled `speedj_rt_stream` to 66.7 Hz (2 × 33.3) — which looked exactly like a new cadence bug in fresh code.

### `libEGL warning: egl: failed to create dri2 screen`

The launch file forces `__GLX_VENDOR_LIBRARY_NAME=nvidia` (same reason as `cameras.launch.py` for the k4a depth engine: GLVND otherwise advertises Mesa/llvmpipe, software rendering). The warning can persist harmlessly — watch RTF, not this message.

---

## Files

```
leeloo_gazebo/
├── launch/leeloo_gazebo.launch.py     # XML surgery + orchestration (detailed module docstring)
├── config/leeloo_controllers.yaml     # ros2_control controllers
├── worlds/leeloo_world.sdf            # world + static obstacles for the ESDF
└── leeloo_gazebo/
    ├── dsr_sim_bridge.py              # stands in for dsr_controller2
    └── base_sim_bridge.py             # stands in for the Clearpath firmware
```

The world contains three static obstacles (table, pillar, box) placed in the arm's workspace and in the Kinect's nominal field of view: without them the ESDF stays empty and obstacle avoidance is not observable.

## See also

- Docstring of [`launch/leeloo_gazebo.launch.py`](launch/leeloo_gazebo.launch.py) — the *why* behind each choice, closest to the code
- `CLAUDE.md` at the root of `src/` — overall architecture of Leeloo and `curobo_ros`
- `curobo_ros/docs/concepts/ros-interfaces.md` — exhaustive inventory of cuRobo's interfaces
