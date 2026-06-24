FROM curobo_ros:aarch64-humble

# Add camera azure kinect
# NOTE (host only): increase USB buffer on Jetson before using the Kinect:
#   echo 'usbcore.usbfs_memory_mb=1000' >> /boot/extlinux/extlinux.conf && reboot

# libsoundio1 dependency — not in Ubuntu 22.04 Jammy, fetch from focal via ports.ubuntu.com (arm64)
RUN apt update && apt install -y software-properties-common curl \
                    ros-humble-librealsense2* \
                    ros-humble-realsense2-* && \
                    apt-add-repository -y -n 'deb http://ports.ubuntu.com/ubuntu-ports focal main' && \
                    apt-add-repository -y 'deb http://ports.ubuntu.com/ubuntu-ports focal universe'
RUN apt-get install -y libsoundio1
RUN apt-add-repository -r -y -n 'deb http://ports.ubuntu.com/ubuntu-ports focal universe' && \
    apt-add-repository -r -y 'deb http://ports.ubuntu.com/ubuntu-ports focal main'

# libk4a1.4 arm64 — Microsoft multiarch repo (Ubuntu 18.04), only version with arm64 packages
# libk4a1.3 and libk4abt are NOT available for arm64
RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/multiarch/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.2_arm64.deb > /tmp/libk4a1.4_1.4.2_arm64.deb
RUN echo 'libk4a1.4 libk4a1.4/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | debconf-set-selections
RUN dpkg -i /tmp/libk4a1.4_1.4.2_arm64.deb

RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/multiarch/prod/pool/main/libk/libk4a1.4-dev/libk4a1.4-dev_1.4.2_arm64.deb > /tmp/libk4a1.4-dev_1.4.2_arm64.deb
RUN dpkg -i /tmp/libk4a1.4-dev_1.4.2_arm64.deb

# Replace depth engine with the ARM64 build from the alpha NuGet package.
# The standard libk4a1.4 depth engine requires OpenCL GPU, which is unavailable
# on JetPack 6.x (NVIDIA removed OpenCL from Orin). The 1.4.0-alpha.4 package
# ships a depth engine compiled for ARM64 without that dependency.
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

# Prevent "dubious ownership" errors on volume-mounted source directories.
# Mounted volumes are owned by the host user (uid≠0) but the container runs
# as root, so git refuses to operate on them without this exception.
RUN git config --global --add safe.directory '*'

# Add azure, doosan, tool_box and pcd_fuse ros2 package
WORKDIR /home/ros2_ws/src

RUN git clone -b humble https://github.com/doosan-robotics/doosan-robot2.git && \
    git clone -b humble https://github.com/microsoft/Azure_Kinect_ROS_Driver.git && \
    git clone https://github.com/Lab-CORO/robotiq_85_gripper.git

# NOTE: this patch targets libk4a1.3 API — verify line number if build fails with libk4a1.4
RUN sed -i '771d' /home/ros2_ws/src/Azure_Kinect_ROS_Driver/src/k4a_ros_device.cpp

RUN apt-get update && apt-get install -y \
        libpoco-dev libyaml-cpp-dev \
        ros-humble-control-msgs ros-humble-realtime-tools ros-humble-xacro \
        ros-humble-joint-state-publisher-gui ros-humble-ros2-control \
        ros-humble-ros2-controllers ros-humble-moveit-msgs \
        dbus-x11 ros-humble-moveit-configs-utils ros-humble-moveit-ros-move-group \
        ros-humble-image-proc \
        && pip3 install pyserial


WORKDIR /home/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-skip dsr_gazebo2"

ENTRYPOINT ["/docker_entrypoint.sh"]
CMD ["bash"]
