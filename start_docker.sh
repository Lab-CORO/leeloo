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
    # Assurez-vous que le serveur X11 autorise les connexions depuis Docker
    xhost +local:docker

    # Exécutez le conteneur Docker avec les bonnes options
    docker run --name leeloo_docker -it \
        --privileged \
        -e NVIDIA_DISABLE_REQUIRE=1 \
        -e NVIDIA_DRIVER_CAPABILITIES=all \
        --device=/dev/:/dev/ \
        --hostname ros1-docker \
        --add-host ros1-docker:127.0.0.1 \
        --gpus all \
        --network host \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix \
        -v ./curobo_ros:/home/ros2_ws/src/curobo_ros\
        -v ./curobo_rviz:/home/ros2_ws/src/curobo_rviz\
        -v ./curobo_msgs:/home/ros2_ws/src/curobo_msgs\
        -v ./pointcloud_fusion:/home/ros2_ws/src/pointcloud_fusion\
        -v ./tool_box:/home/ros2_ws/src/tool_box \
        -v ./leeloo:/home/ros2_ws/src/leeloo \
        --cap-add=sys_nice \
        --ulimit rtprio=99 \
        --ulimit memlock=-1 \
        leeloo_docker:x86 
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




