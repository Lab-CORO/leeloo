FROM curobo_ros:aarch64-humble

# ── System / hardware layer (identical to dev image) ─────────────────────────

# libsoundio1 — not in Ubuntu 22.04 Jammy, pulled from focal
RUN apt update && apt install -y software-properties-common curl \
        ros-humble-librealsense2* ros-humble-realsense2-* && \
    apt-add-repository -y -n 'deb http://ports.ubuntu.com/ubuntu-ports focal main' && \
    apt-add-repository -y 'deb http://ports.ubuntu.com/ubuntu-ports focal universe'
RUN apt-get install -y libsoundio1
RUN apt-add-repository -r -y -n 'deb http://ports.ubuntu.com/ubuntu-ports focal universe' && \
    apt-add-repository -r -y 'deb http://ports.ubuntu.com/ubuntu-ports focal main'

# libk4a1.4 arm64 (Microsoft multiarch repo, Ubuntu 18.04 — only version with arm64)
RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/multiarch/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.2_arm64.deb \
        -o /tmp/libk4a1.4_1.4.2_arm64.deb
RUN echo 'libk4a1.4 libk4a1.4/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' \
        | debconf-set-selections
RUN dpkg -i /tmp/libk4a1.4_1.4.2_arm64.deb
RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/multiarch/prod/pool/main/libk/libk4a1.4-dev/libk4a1.4-dev_1.4.2_arm64.deb \
        -o /tmp/libk4a1.4-dev_1.4.2_arm64.deb
RUN dpkg -i /tmp/libk4a1.4-dev_1.4.2_arm64.deb

# ARM64 depth engine from 1.4.0-alpha.4 NuGet (no OpenCL — required on JetPack 6.x)
RUN apt-get install -y unzip xvfb libgl1-mesa-dri && \
    curl -sSL https://www.nuget.org/api/v2/package/Microsoft.Azure.Kinect.Sensor/1.4.0-alpha.4 \
        -o /tmp/k4a-alpha.nupkg && \
    unzip -jo /tmp/k4a-alpha.nupkg \
        "linux/lib/native/arm64/release/libdepthengine.so.2.0" \
        -d /usr/lib/aarch64-linux-gnu/libk4a1.4/ && \
    rm /tmp/k4a-alpha.nupkg

COPY 99-k4a.rules /etc/udev/rules.d/99-k4a.rules
COPY docker_entrypoint.sh /docker_entrypoint.sh
RUN chmod +x /docker_entrypoint.sh

RUN git config --global --add safe.directory '*'

# ── ROS workspace apt dependencies ───────────────────────────────────────────

RUN apt-get update && apt-get install -y \
        libpoco-dev libyaml-cpp-dev \
        ros-humble-control-msgs ros-humble-realtime-tools ros-humble-xacro \
        ros-humble-joint-state-publisher-gui ros-humble-ros2-control \
        ros-humble-ros2-controllers ros-humble-moveit-msgs \
        dbus-x11 ros-humble-moveit-configs-utils ros-humble-moveit-ros-move-group \
        ros-humble-image-proc \
    && pip3 install pyserial

# ── Source packages ───────────────────────────────────────────────────────────

WORKDIR /home/ros2_ws/src

# Third-party packages (same as dev image)
RUN git clone -b humble https://github.com/doosan-robotics/doosan-robot2.git && \
    git clone -b humble https://github.com/microsoft/Azure_Kinect_ROS_Driver.git && \
    git clone https://github.com/Lab-CORO/robotiq_85_gripper.git

# Patch Azure_Kinect_ROS_Driver for libk4a1.4 API (removes incompatible line 771)
RUN sed -i '771d' Azure_Kinect_ROS_Driver/src/k4a_ros_device.cpp

# Lab-CORO external packages (from my.repos, with user-confirmed branch overrides)
# Remove any pre-existing clones from the base image before re-cloning at the
# correct branches.
RUN rm -rf curobo_ros curobo_msgs curobo_rviz tool_box pointcloud_fusion \
           ros2_handeye_calibration ros2_markertracker
RUN git clone https://github.com/Lab-CORO/curobo_ros.git && \
    git clone https://github.com/Lab-CORO/curobo_msgs.git && \
    git clone https://github.com/Lab-CORO/curobo_rviz.git && \
    git clone https://github.com/Lab-CORO/tool_box.git && \
    git clone https://github.com/Lab-CORO/pointcloud_fusion.git && \
    git clone -b HandEyeCalibUpdates https://github.com/Lab-CORO/ros2_handeye_calibration.git && \
    git clone -b MarkerUpdates https://github.com/Lab-CORO/ros2_markertracker.git

# tool_box bundles a copy of ros2_markertracker; hide it from colcon so only the
# standalone clone on MarkerUpdates is used.
RUN touch tool_box/ros2_markertracker/COLCON_IGNORE

# Leeloo native packages — clone the workspace repo and extract only the 3 ROS
# packages (the repo root also holds the 7 external repos as sibling dirs, so
# we copy selectively to avoid collisions in the colcon src tree).
# Config files (calibration_poses.yaml, kinect_hand_eye_result.yaml, etc.) are
# embedded inside these packages and are baked in here.
RUN git clone https://github.com/Lab-CORO/leeloo.git /tmp/leeloo_ws && \
    cp -r /tmp/leeloo_ws/leeloo . && \
    cp -r /tmp/leeloo_ws/leeloo_calibration . && \
    cp -r /tmp/leeloo_ws/leeloo_msgs . && \
    rm -rf /tmp/leeloo_ws

# ── Build ─────────────────────────────────────────────────────────────────────

WORKDIR /home/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-skip dsr_gazebo2"

ENTRYPOINT ["/docker_entrypoint.sh"]
CMD ["bash"]
