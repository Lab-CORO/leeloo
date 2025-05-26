# Leeloo 🤖 – Mobile Manipulator ROS 2 Workspace

Repository for **Leeloo**, CoRo-Lab’s mobile-manipulator research platform.
The robot combines:

* a 6-DoF collaborative **arm** (UR5e by default, plug-and-play for others)
* a holonomic **mobile base** (Clearpath Ridgeback-class)
* up-to four **RGB-D cameras** (Azure Kinect out-of-the-box)
* an onboard x86 PC with an NVIDIA GPU for realtime perception & motion-planning

The stack is built around **ROS 2 Humble**, CuRobo for optimisation-based control, and containerised with Docker for reproducible deployments.
---

## Table of Contents

1. [Quick start](#quick-start)
2. [Hardware & network setup](#hardware--network-setup)
3. [Building the workspace](#building-the-workspace)
4. [Running Leeloo](#running-leeloo)
5. [Repository layout](#repository-layout)
6. [Troubleshooting](#troubleshooting)

---

## Quick start

```bash
# 1. Clone the repository
git clone https://github.com/Lab-CORO/leeloo.git
cd leeloo

# 2. Install vcs and pull external ROS packages
sudo apt update && sudo apt install python3-vcstool
vcs import < my.repos --recursive        # pulls external sources

# 3. Build & start the container
./build_docker.sh     # one-time image build (≈25 min on first run)
./start_docker.sh     # drops you in /workspaces/leeloo inside the container
```

> **Tip:** Need another terminal?
> `docker exec -it leeloo_docker bash`

---

## Hardware & network setup

| Component         | Default IP / Port | Notes                                             |
| ----------------- | ----------------- | ------------------------------------------------- |
| Robot PC (Host)   | `192.168.137.20`  | Static address on USB-C tether                    |
| UR arm controller | `192.168.137.100` | Set in teach pendant                              |
| Ridgeback base    | `192.168.137.101` | DHCP or static                                    |
| Azure Kinects     | USB-C             | Udev rule `99-k4a.rules` sets correct permissions |

*Connect the USB-C tether before launching; the launch files assume the above addressing.*

---

## Building the workspace

Inside the **running container**:

```bash
# Source ROS and build
source /opt/ros/humble/setup.bash
colcon build --symlink-install
# Add to every new terminal
source install/setup.bash
```

---

## Running Leeloo

Launch everything—perception, CuRobo motion-planning, arm, base & TF—using a single command:

```bash
ros2 launch leeloo start_system.launch.py
```

### Typical launch arguments

| Argument              | Default | Description                   |
| --------------------- | ------- | ----------------------------- |
| `use_sim_time:=false` | false   | Switch to Gazebo/ignition sim |
| `joystick:=false`     | false   | Enable game-pad tele-op       |
| `record:=false`       | false   | Rosbag record all topics      |

---

## Repository layout

```
leeloo/
├── leeloo/                # ROS 2 packages (core nodes, launch, rviz)
├── my.repos               # External dependencies for vcs
├── x86.dockerfile         # Base image (Ubuntu 22.04 + CUDA + ROS 2)
├── build_docker.sh        # Wrapper to build the image
├── start_docker.sh        # Runs the container with proper mounts & devices
└── 99-k4a.rules           # Udev permissions for Azure Kinect
```

Primary code is in **`leeloo/`**; follow ROS 2 package conventions (src, launch, config, msgs).

---

## Troubleshooting

| Symptom                              | Fix                                                                                                                       |
| ------------------------------------ | ------------------------------------------------------------------------------------------------------------------------- |
| *Docker build fails on CuRobo layer* | Ensure you first built/pulled [`curobo_ros`](https://github.com/Lab-CORO/curobo_ros) image and are on a CUDA-capable GPU. |
| *No camera topics*                   | Check udev rule installed. Un-/re-plug Azure Kinect.                                                                      |
| *Cannot ping robot arm*              | Verify static IPs match table above; firewalls off.                                                                       |
| *RViz TF-tree broken*                | Launch `ros2 run tf2_tools view_frames` and check if `/tf_static` is published by the base.                               |

---

## Contributing


---



