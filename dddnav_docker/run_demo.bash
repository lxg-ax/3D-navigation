#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

xhost +local:docker 2>/dev/null || true

is_x64=$(docker image ls dddnav | grep x64 || true)
is_cuda=$(docker image ls dddnav | grep cuda || true)
is_l4t_r36=$(docker image ls dddnav | grep l4t_r36 || true)
if [ "$is_cuda" != "" ] ;then 
    docker run -it \
        --privileged \
        --network=host \
        --gpus=all \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="NVIDIA_VISIBLE_DEVICES=all"\
        --env="NVIDIA_DRIVER_CAPABILITIES=all"\
        --volume="/dev:/dev" \
        --volume="/tmp:/tmp" \
        --volume="${HOME}/dddnav_bags:/root/dddnav_bags" \
        --volume="${REPO_ROOT}:/root/dddnav_navigation" \
        --name="dddnav_humble_dev" \
        dddnav:cuda
elif [ "$is_x64" != "" ] ;then 
    docker run -it \
        --privileged \
        --network=host \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/dev:/dev" \
        --volume="/tmp:/tmp" \
        --volume="${HOME}/dddnav_bags:/root/dddnav_bags" \
        --volume="${REPO_ROOT}:/root/dddnav_navigation" \
        --name="dddnav_humble_dev" \
        dddnav:x64
elif [ "$is_l4t_r36" != "" ] ;then 
    docker run -it \
        --privileged \
        --network=host \
        --runtime=nvidia\
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --env="NVIDIA_VISIBLE_DEVICES=all"\
        --env="NVIDIA_DRIVER_CAPABILITIES=all"\
        --volume="/dev:/dev" \
        --volume="/tmp:/tmp" \
        --volume="${HOME}/dddnav_bags:/root/dddnav_bags" \
        --volume="${REPO_ROOT}:/root/dddnav_navigation" \
        --name="dddnav_humble_dev" \
        dddnav:l4t_r36
else
    echo "No dddnav image found (tried cuda, x64, l4t_r36). Build with:"
    echo "  cd ${SCRIPT_DIR}/docker_file && ./build.bash"
    exit 1
fi
