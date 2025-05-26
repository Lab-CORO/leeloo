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


# This script will create a dev docker. Run this script by calling `bash build_dev_docker.sh`
# If you want to build a isaac sim docker, run this script with `bash build_dev_docker.sh isaac`

# Check architecture to build:
image_tag="x86"
if 
[ -n "$(docker images -q curobo_docker:x86)" ]
then
    echo "Docker image curobo_docker:x86 already exists"
else
    echo "Docker image curobo_docker:x86 does not exist"
    echo "Building docker image curobo_docker:x86"
    bash ../docker_curobo/build_docker.sh
fi

docker build  -t leeloo_docker:${image_tag} -f "x86.dockerfile" . 
