# Leeloo — Mobile Manipulator ROS 2 Workspace

Repository for **Leeloo**, CoRo-Lab's mobile-manipulator research platform.

* Holonomic mobile base: **Clearpath Ridgeback** (namespace `/r100_0597`)
* 6-DoF collaborative arm: **Doosan M1013** (namespace `dsr01`)
* Gripper: **Robotiq 2F-85**
* Depth cameras: **Azure Kinect** (fixed, on a mast) + **OAK-D SR** (eye-in-hand, on the wrist)
* Onboard compute: **NVIDIA Jetson Orin** (aarch64), GPU-accelerated perception & motion planning
* **ROS 2 Jazzy**, containerized with Docker

Motion planning is delegated to [cuRobo](https://github.com/Lab-CORO/curobo_ros) (GPU-accelerated, MPC-based), driven through this workspace's own execution and TF layer.

> This README documents the platform as it currently runs (Doosan/Jazzy/Jetson). See [Known discrepancies](#known-discrepancies) at the bottom for places where other files in this repo (Dockerfiles, an older README revision) have not caught up with the code.

---

## Table of Contents

- [Architecture](#architecture)
- [The unified `/leeloo` TF tree](#the-unified-leeloo-tf-tree)
- [Main interfaces](#main-interfaces)
- [Hardware & network](#hardware--network)
- [Installation](#installation)
- [Building the workspace](#building-the-workspace)
- [Running Leeloo](#running-leeloo)
- [Repository layout](#repository-layout)
- [Known discrepancies](#known-discrepancies)

---

## Architecture

Command chain, arm and base, real hardware (`sim:=false`, the default):

```
cuRobo MPC (curobo_trajectory_planner)
  └─▶ /leeloo/execute_trajectory        (trajectory_msgs/JointTrajectory)
        └─▶ leeloo/execute_trajectory (node)
              └─▶ /dsr01/dsr_controller2/speedj_rt_stream (dsr_msgs2/SpeedjRtStream)
                    └─▶ dsr_controller2 → DRFL speedj_rt() → arm hardware

/cmd_vel (geometry_msgs/Twist)
  └─▶ leeloo/ridgeback_control (node)
        └─▶ /r100_0597/joy_teleop/cmd_vel (geometry_msgs/TwistStamped)
              └─▶ Clearpath firmware → base hardware

Kinect + OAK-D
  └─▶ robot_segmentation (curobo_ros)
        └─▶ /masked_depth_image, /masked_depth_image_oak_wrist
              └─▶ cuRobo ESDF (obstacle map used by the MPC)
```

`leeloo_gazebo` substitutes every hardware-facing block (arm driver, base firmware, both cameras) with a Gazebo Sim equivalent that exposes the exact same topics and types — nothing above the substitution line changes when `sim:=true`. See [leeloo_gazebo/README.md](leeloo_gazebo/README.md).

## The unified `/leeloo` TF tree

Leeloo's arm, base, structure and cameras are published as separate ROS graphs (different machines, different drivers). `bringup_leeloo.launch.py` unifies them into one tree on `/leeloo/tf` and `/leeloo/tf_static`:

| Source | Mechanism |
|---|---|
| Ridgeback (`/r100_0597/tf[_static]`, separate machine) | `topic_tools/relay` → `/leeloo/tf[_static]` |
| Doosan arm + Kinect driver (`/tf[_static]`, global) | `topic_tools/relay` → `/leeloo/tf[_static]` |
| Structure bridge (`default_mount → … → dsr01/world`) | `robot_state_publisher`, written directly |
| Camera pose (`base_link → camera_base`) | `kinect_tf_computation_node`, written directly |

Every Leeloo-side node (structure RSP, calibration, cuRobo, segmentation, marker tracking, hand-eye) remaps its own `/tf` and `/tf_static` onto `/leeloo/tf[_static]` via `tf_remap` / `tf_static_remap` launch arguments — a single source of truth propagated from `bringup_leeloo.launch.py`. Data topics (images, `dsr01` services, …) stay global and unremapped. Arm frames are prefixed `dsr01/` (`dsr01/world`, `dsr01/link_1..6`) to avoid a `base_link` collision with the Ridgeback.

See the header of [leeloo/launch/bringup_leeloo.launch.py](leeloo/launch/bringup_leeloo.launch.py) for the full rationale.

## Main interfaces

| Interface | Type | Role |
|---|---|---|
| `/leeloo/execute_trajectory` | `trajectory_msgs/JointTrajectory` | Arm trajectory to stream to the driver |
| `/cmd_vel` | `geometry_msgs/Twist` | Base velocity command |
| `/robotiq_gripper_controller/gripper_cmd` | `control_msgs/action/ParallelGripperCommand` | Gripper open/close |
| `/curobo_trajectory_planner/execute_trajectory` | `curobo_msgs/action/SendTrajectory` | Plan + execute to a target pose (cuRobo) |
| `/curobo_trajectory_planner/mpc_goal` | `geometry_msgs/Pose` | Live goal update while the MPC is servoing |
| `/ExecuteTrajectory/enable_streaming` | `std_srvs/SetBool` | Toggle: leeloo pipeline vs. direct Doosan API control |
| `/run_calibration` | `leeloo_msgs/action/RunCalibration` | Hand-eye calibration procedure |
| `/leeloo/tf`, `/leeloo/tf_static` | `tf2_msgs/TFMessage` | Unified TF tree (see above) |

Full per-package interfaces are documented in each package's own README (see [Repository layout](#repository-layout)).

## Hardware & network

| Component | Address | Notes |
|---|---|---|
| Doosan arm controller | `192.168.50.100` (`host`) | Set via `control.launch.py`'s `host` argument |
| Doosan RT interface | `192.168.50.50` (`rt_host`) | `control.launch.py`'s `rt_host` argument |
| Ridgeback base | namespace `/r100_0597` | Separate onboard computer, own TF/odom namespace |
| Azure Kinect + OAK-D SR | USB | Udev rule [99-k4a.rules](99-k4a.rules) sets Kinect permissions |

## Installation

```bash
git clone https://github.com/Lab-CORO/leeloo.git
cd leeloo

# Pull external ROS packages declared in my.repos
sudo apt update && sudo apt install python3-vcstool
vcs import < my.repos --recursive
```

[my.repos](my.repos) currently pulls:

| Package | Purpose |
|---|---|
| [`curobo_ros`](https://github.com/Lab-CORO/curobo_ros) | GPU trajectory planning / MPC, exposed as ROS 2 services & actions |
| [`curobo_msgs`](https://github.com/Lab-CORO/curobo_msgs) | Interfaces consumed by `curobo_ros` |
| [`curobo_rviz`](https://github.com/Lab-CORO/curobo_rviz) | RViz2 plugins for cuRobo |
| [`ros2_handeye_calibration`](https://github.com/Lab-CORO/ros2_handeye_calibration) (branch `HandEyeCalibUpdates`) | Hand-eye calibration service (`easy_handeye2`) |
| [`ros2_markertracker`](https://github.com/Lab-CORO/ros2_markertracker) (branch `MarkerUpdates`) | ArUco/AprilTag marker detection |

Build and start the container — **confirm the exact commands with the maintainer**, see [Known discrepancies](#known-discrepancies) below:

```bash
./build_docker.sh     # builds x86.dockerfile → tag leeloo_docker:x86
./start_docker.sh     # runs leeloo_docker:aarch64-jazzy — tag mismatch, see below
docker exec -it leeloo_docker bash
```

## Building the workspace

Inside the running container:

```bash
cd /home/ros2_ws
source /opt/ros/humble/setup.bash    # or the ROS 2 Jazzy setup script actually shipped in the image — to confirm
colcon build
source install/setup.bash
```

## Running Leeloo

```bash
ros2 launch leeloo bringup_leeloo.launch.py
```

### Launch arguments (`bringup_leeloo.launch.py`)

| Argument | Default | Description |
|---|---|---|
| `sim` | `false` | `true`: replace hardware with `leeloo_gazebo` (arm/base/camera drivers, gripper controller). Rest of the stack unchanged. |
| `enabled_cameras` | `kinect,oak` | Comma-separated camera list (`kinect`, `oak`, `kinect,oak`, or `none`) |
| `enable_calibration` | `true` | Starts the hand-eye calibration chain (marker trackers, `handeye_server`, `auto_calibration_server`). Set to `false` in production — without a calibration marker in view, `handeye_server` never creates its services and logs an ERROR every ~10 s. |
| `tf_remap` / `tf_static_remap` | `/leeloo/tf` / `/leeloo/tf_static` | Single source of truth for the unified TF tree, see above |

Arguments propagated into `control.launch.py` (Doosan connection, cuRobo planner, MPC diagnostics): `host`, `rt_host`, `port`, `model`, `mode`, `gui`, `voxel_size`, `mpc_debug`, `diag_dir`.

Common variants:

```bash
# Production run, no calibration chain
ros2 launch leeloo bringup_leeloo.launch.py enable_calibration:=false

# Full stack in Gazebo simulation
ros2 launch leeloo bringup_leeloo.launch.py sim:=true enable_calibration:=false

# MPC timing tests — cameras are the dominant simulated cost
ros2 launch leeloo bringup_leeloo.launch.py sim:=true enable_calibration:=false enabled_cameras:=none
```

## Repository layout

```
leeloo/
├── leeloo/               # Core package — nodes, launch files, URDF, config    → leeloo/README.md
├── leeloo_msgs/           # Custom action/service definitions                   → leeloo_msgs/README.md
├── leeloo_calibration/     # Hand-eye calibration nodes                          → leeloo_calibration/README.md
├── leeloo_gazebo/          # Gazebo Sim hardware substitute                      → leeloo_gazebo/README.md
├── leeloo_rviz/            # RViz2 visualization session (launch-only)           → leeloo_rviz/README.md
├── my.repos                # External ROS package dependencies (vcs)
├── *.dockerfile            # Docker images (x86, aarch64, aarch64_prod, aarch64_jazzy)
├── build_docker.sh          # Build wrapper
├── start_docker.sh          # Runs the container with device/GPU mounts
└── 99-k4a.rules             # Udev permissions for Azure Kinect
```

## Known discrepancies

Found while writing this documentation, listed here rather than silently "fixed" in the docs so nothing gets papered over — **no code was changed**.

- **Docker image tag mismatch.** [build_docker.sh](build_docker.sh) builds [x86.dockerfile](x86.dockerfile) and tags it `leeloo_docker:x86`. [start_docker.sh](start_docker.sh) runs `leeloo_docker:aarch64-jazzy`, which nothing in this repo builds under that exact tag. Four Dockerfiles coexist ([x86](x86.dockerfile), [aarch64](aarch64.dockerfile), [aarch64_prod](aarch64_prod.dockerfile), [aarch64_jazzy](aarch64_jazzy.dockerfile)) — confirm which one is actually current for the Jetson.
- **`start_docker.sh` mounts missing sources.** It bind-mounts `./tool_box` and `./pointcloud_fusion`, both commented out / absent from [my.repos](my.repos).
- **[leeloo/src/grasp.cpp](leeloo/src/grasp.cpp) is not built.** Despite its `.cpp` extension it is a Python ROS 2 node (state-machine pick sequence). It is absent from [leeloo/CMakeLists.txt](leeloo/CMakeLists.txt), so `ros2 run leeloo grasp` does not exist as installed today. It also imports `heron_interface`, which is declared in neither `leeloo/package.xml` nor `my.repos`.
- **[leeloo/config/config.yaml](leeloo/config/config.yaml) is empty** (0 bytes).
- **[leeloo/package.xml](leeloo/package.xml)** still has `description` and `license` set to `TODO`.

Commands throughout this documentation set are marked for verification — see the checklist shared separately with the maintainer.
