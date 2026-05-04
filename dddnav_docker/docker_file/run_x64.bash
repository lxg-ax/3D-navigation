#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# dddnav_docker/docker_file -> workspace root
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

xhost +local:docker 2>/dev/null || true

docker run -it \
    --privileged \
    --network=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp:/tmp" \
    --volume="/dev:/dev" \
    --volume="${REPO_ROOT}:/root/dddnav_navigation" \
    --volume="${HOME}/dddnav_bags:/root/dddnav_bags" \
    --name="dddnav_x64" \
    dddnav:x64
