#!/bin/bash
##
## Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
##
## NVIDIA CORPORATION, its affiliates and licensors retain all intellectual
## property and proprietary rights in and to this material, related
## documentation and any modifications thereto. Any use, reproduction,
## disclosure or distribution of this material and related documentation
## without an express license agreement from NVIDIA CORPORATION or
## its affiliates is strictly prohibited.
##

# Check if the branch argument is empty
if [ -z "$1" ]; then
    echo "Branch argument empty, sending default branch: main"
    branch_arg="main"
else
    branch_arg="$1"
fi

if ! [[ "$OSTYPE" == "msys" ]]; then
    # Allow X11 connections from Docker only if a display is available
    if [ -n "$DISPLAY" ]; then
        xhost +local:docker
    fi

    # The container entrypoint starts its own Xvfb virtual display on :99.
    # Do not forward the host display: the Kinect depth engine uses software
    # OpenGL (Mesa LLVMpipe) and does not need a hardware X server.
    DISPLAY_FLAGS=""


    # Exécutez le conteneur Docker avec les bonnes options
    docker run --name leeloo_docker -it \
        --privileged \
        -e NVIDIA_DISABLE_REQUIRE=1 \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        --device=/dev/:/dev/ \
        -v /dev/bus/usb:/dev/bus/usb \
        --hostname ros1-docker \
        --add-host ros1-docker:127.0.0.1 \
        --gpus all \
        --network host \
        $DISPLAY_FLAGS \
        -v ./curobo_ros:/home/ros2_ws/src/curobo_ros\
        -v ./curobo_rviz:/home/ros2_ws/src/curobo_rviz\
        -v ./curobo_msgs:/home/ros2_ws/src/curobo_msgs\
        -v ./pointcloud_fusion:/home/ros2_ws/src/pointcloud_fusion\
        -v ./tool_box:/home/ros2_ws/src/tool_box \
        -v ./ros2_handeye_calibration:/home/ros2_ws/src/ros2_handeye_calibration \
        -v ./ros2_markertracker:/home/ros2_ws/src/ros2_markertracker \
        -v ./leeloo:/home/ros2_ws/src/leeloo \
        -v ./leeloo_calibration:/home/ros2_ws/src/leeloo_calibration \
        -v ./leeloo_msgs:/home/ros2_ws/src/leeloo_msgs \
        -v /usr/lib/aarch64-linux-gnu/tegra:/usr/lib/aarch64-linux-gnu/tegra \
        --cap-add=sys_nice \
        --ulimit rtprio=99 \
        --ulimit memlock=-1 \
        leeloo_docker:aarch64-jazzy
else
    echo "Detected OS is msys, make sure to have an X server running on your host machine"
    # Exécutez seulement le conteneur Docker avec les options appropriées
    docker run --name leeloo_docker --rm -it \
        --privileged \
        -e NVIDIA_DISABLE_REQUIRE=1 \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        --hostname ros1-docker \
        --add-host ros1-docker:127.0.0.1 \
        --gpus all \
        --network host \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        leeloo_docker:x86 
fi




