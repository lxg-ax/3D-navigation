#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

xhost +local:docker 2>/dev/null || true

is_cuda=$(docker image ls dddnav | grep cuda || true)
is_l4t_r36=$(docker image ls dddnav | grep l4t_r36 || true)

if [ "$is_cuda" != "" ] ;then
    echo "Detect image of dddnav:cuda"
    echo "Enter ROS_DOMAIN_ID you want for the container."
    read domain_id
    docker run -it \
        --privileged \
        --network=host \
        --gpus=all \
        --env="NVIDIA_VISIBLE_DEVICES=all"\
        --env="NVIDIA_DRIVER_CAPABILITIES=all"\
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="ROS_DOMAIN_ID=${domain_id}" \
        --volume="/tmp:/tmp" \
        --volume="/dev:/dev" \
        --volume="${HOME}/dddnav_bags:/root/dddnav_bags" \
        --volume="${REPO_ROOT}:/root/dddnav_navigation" \
        --name="dddnav_humble_cuda_dev" \
        dddnav:cuda
elif [ "$is_l4t_r36" != "" ] ;then 
    echo "Detect image of dddnav:l4t_r36"
    echo "Enter ROS_DOMAIN_ID you want for the container."
    read domain_id
    docker run -it \
        --privileged \
        --network=host \
        --runtime=nvidia\
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="NVIDIA_VISIBLE_DEVICES=all"\
        --env="NVIDIA_DRIVER_CAPABILITIES=all"\
        --env="ROS_DOMAIN_ID=${domain_id}" \
        --volume="/dev:/dev" \
        --volume="/tmp:/tmp" \
        --volume="${HOME}/dddnav_bags:/root/dddnav_bags" \
        --volume="${REPO_ROOT}:/root/dddnav_navigation" \
        --name="dddnav_humble_l4t_dev" \
        dddnav:l4t_r36
else
    echo "No matching image found. Build one first, e.g.:"
    echo "  cd ${SCRIPT_DIR} && ./build.bash"
    echo "Expected tags: dddnav:cuda (x64+GPU) or dddnav:l4t_r36 (Jetson)."
    exit 1
fi
