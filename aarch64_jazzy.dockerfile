FROM curobo_ros:aarch64-jazzy

# Add camera azure kinect
# NOTE (host only): increase USB buffer on Jetson before using the Kinect:
#   echo 'usbcore.usbfs_memory_mb=1000' >> /boot/extlinux/extlinux.conf && reboot

# libsoundio1 — available in Ubuntu 24.04 Noble universe (no focal workaround needed)
RUN apt-get update && apt-get install -y software-properties-common curl \
        libsoundio1 \
        ros-jazzy-librealsense2* \
        ros-jazzy-realsense2-* \
    && rm -rf /var/lib/apt/lists/* || true

# Add camera OAK-D SR (depthai-ros v3)
RUN apt-get update && apt-get install -y \
        ros-jazzy-depthai-ros-v3 \
    && rm -rf /var/lib/apt/lists/* || true

# ── libusb SANS backend udev, pour l'OAK-D ───────────────────────────────────
# Sans ça, le driver boucle sur :
#   "Failed to find device after booting: X_LINK_DEVICE_NOT_FOUND"
# L'OAK-D se ré-énumère sur le bus USB quand il boote (idProduct 2485 → f63b).
# Le libusb de Noble est compilé AVEC le backend udev, qui apprend ce genre
# d'événement hotplug via le démon udev — démon qui ne tourne PAS dans un
# conteneur. libusb ne voit donc jamais la caméra réapparaître. Compilé
# --disable-udev, il scanne directement /dev/bus/usb (bind-monté) et la retrouve.
# C'est pour la même raison que le wheel Python depthai fonctionne : les wheels
# manylinux embarquent un libusb sans udev.
# NB : c'est le backend qui compte, pas la version — 1.0.28 compilé AVEC udev
# échoue exactement comme le 1.0.27 système (vérifié).
# Installé dans /opt/libusb-noudev, hors du ldconfig système : injecté uniquement
# dans le driver OAK par LD_PRELOAD depuis bringup_leeloo.launch.py, pour ne pas
# changer le libusb que voit l'Azure Kinect (qui, lui, fonctionne tel quel).
RUN apt-get update && apt-get install -y --no-install-recommends wget && \
    wget -q https://github.com/libusb/libusb/releases/download/v1.0.28/libusb-1.0.28.tar.bz2 \
        -O /tmp/libusb.tar.bz2 && \
    tar -xjf /tmp/libusb.tar.bz2 -C /tmp && \
    cd /tmp/libusb-1.0.28 && \
    ./configure --prefix=/opt/libusb-noudev --disable-static --disable-udev && \
    make -j"$(nproc)" && make install && \
    cd / && rm -rf /tmp/libusb-1.0.28 /tmp/libusb.tar.bz2 && \
    rm -rf /var/lib/apt/lists/*

# Confort pour les tests manuels (docker exec -it ... bash) : LD_PRELOAD posé au
# lancement du shell, uniquement pour les sessions interactives. bringup_leeloo
# le fait déjà lui-même, scopé au seul driver OAK (GroupAction) — ce .bashrc ne
# le remplace pas, il évite juste de le taper à la main en test manuel.
RUN echo '' >> /root/.bashrc && \
    echo '# OAK-D SR : libusb sans backend udev (fix X_LINK_DEVICE_NOT_FOUND en conteneur)' >> /root/.bashrc && \
    echo 'export LD_PRELOAD=/opt/libusb-noudev/lib/libusb-1.0.so.0' >> /root/.bashrc

# libk4a1.4 arm64 — Microsoft multiarch repo (Ubuntu 18.04), only version with arm64 packages
# libk4a1.3 and libk4abt are NOT available for arm64
RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/multiarch/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.2_arm64.deb > /tmp/libk4a1.4_1.4.2_arm64.deb
RUN echo 'libk4a1.4 libk4a1.4/accepted-eula-hash string 0f5d5c5de396e4fee4c0753a21fee0c1ed726cf0316204edda484f08cb266d76' | debconf-set-selections
RUN dpkg -i /tmp/libk4a1.4_1.4.2_arm64.deb

RUN curl -sSL https://packages.microsoft.com/ubuntu/18.04/multiarch/prod/pool/main/libk/libk4a1.4-dev/libk4a1.4-dev_1.4.2_arm64.deb > /tmp/libk4a1.4-dev_1.4.2_arm64.deb
RUN dpkg -i /tmp/libk4a1.4-dev_1.4.2_arm64.deb

# Replace depth engine with the ARM64 build from the alpha NuGet package.
# The standard libk4a1.4 depth engine requires OpenCL GPU, which is unavailable
# on JetPack 6.x/7.x (NVIDIA removed OpenCL from Orin). The 1.4.0-alpha.4 package
# ships a depth engine compiled for ARM64 without that dependency.
RUN apt-get update && apt-get install -y unzip xvfb libgl1-mesa-dri && \
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
RUN git config --global --add safe.directory '*'

# Add azure, doosan, tool_box and pcd_fuse ros2 package
WORKDIR /home/ros2_ws/src

# doosan-robot2: check if a jazzy branch exists, fall back to humble
RUN git clone -b jazzy https://github.com/doosan-robotics/doosan-robot2.git 2>/dev/null || \
    git clone -b humble https://github.com/doosan-robotics/doosan-robot2.git

RUN git clone -b humble https://github.com/microsoft/Azure_Kinect_ROS_Driver.git && \
    git clone https://github.com/Lab-CORO/robotiq_85_gripper.git

# NOTE: this patch targets libk4a1.4 API (removes incompatible line 771)
RUN sed -i '771d' /home/ros2_ws/src/Azure_Kinect_ROS_Driver/src/k4a_ros_device.cpp

# Jazzy migration: cv_bridge moved its header from cv_bridge/cv_bridge.h to
# .hpp (the .h shim was dropped in ROS 2 Jazzy). The humble-branch driver still
# includes the old path → patch all occurrences in the cloned sources.
RUN grep -rl 'cv_bridge/cv_bridge.h' /home/ros2_ws/src/Azure_Kinect_ROS_Driver/ \
    | xargs -r sed -i 's@cv_bridge/cv_bridge\.h@cv_bridge/cv_bridge.hpp@g'

RUN apt-get update && apt-get install -y \
        libpoco-dev libyaml-cpp-dev \
        ros-jazzy-control-msgs ros-jazzy-realtime-tools ros-jazzy-xacro \
        ros-jazzy-joint-state-publisher-gui ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers ros-jazzy-moveit-msgs \
        dbus-x11 ros-jazzy-moveit-configs-utils ros-jazzy-moveit-ros-move-group \
        ros-jazzy-image-proc \
        ros-jazzy-cv-bridge ros-jazzy-image-transport \
        ros-jazzy-angles ros-jazzy-tf2-geometry-msgs \
        && pip3 install pyserial \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home/ros2_ws

# Résout automatiquement les dépendances système restantes de tous les
# packages clonés (doosan-robot2, Azure_Kinect_ROS_Driver, robotiq). En
# best-effort : K4A vient de libk4a (dpkg), dsr_gazebo2 est ignoré au build.
RUN apt-get update && \
    /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
        rosdep install --from-paths src --ignore-src -y --rosdistro jazzy \
        --skip-keys 'K4A dsr_gazebo2' || \
        echo 'rosdep: clés non résolues ignorées (best-effort)'" && \
    rm -rf /var/lib/apt/lists/*

RUN /bin/bash -c "source /opt/ros/jazzy/setup.bash && \
    colcon build --packages-skip dsr_gazebo2"

# ── RMW : Cyclone DDS (remplace FastDDS) ─────────────────────────────────────
# Meilleur comportement sur gros messages et configuration bien plus simple que
# FastDDS. DOIT être identique sur TOUT le graphe ROS (ce conteneur + capacitynet),
# sinon les nœuds ne se voient pas. ENV s'applique au process principal ET aux
# shells `docker exec`.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0
ENV CYCLONEDDS_URI=/home/ros2_ws/src/capacitynet/config/profile.xml

ENTRYPOINT ["/docker_entrypoint.sh"]
CMD ["bash"]
