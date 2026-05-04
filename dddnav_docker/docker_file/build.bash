#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

function build_x64(){
    docker build --network host -t dddnav:x64 -f Dockerfile_x64 . --no-cache
}

#-----select image
echo -n "Select image type (x64/l4t/x64_gz): "
read -r image_type

if [[ $image_type == "x64" ]]; then
    echo -n "Do you want to build image using cuda? (Y/N): "
    read -r is_cuda
    if [ "$is_cuda" != "${is_cuda#[Yy]}" ] ;then
        echo "----> Creating x64 base image first, then CUDA layer"
        build_x64
        echo "----> Starting second layer with CUDA"
        if ! command -v nvidia-smi &>/dev/null; then
            echo "ERROR: nvidia-smi not found. Install NVIDIA drivers + CUDA-capable GPU, or answer N for CPU-only x64."
            exit 1
        fi
        CUDA_ARCH="$(nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null | head -n1 | tr -d '[:space:]')"
        if [[ -z "${CUDA_ARCH}" ]]; then
            echo "ERROR: Could not read GPU compute_cap from nvidia-smi."
            exit 1
        fi
        echo "Using CUDA_ARCH_BIN=${CUDA_ARCH} (from nvidia-smi)"
        docker build --network host -t dddnav:cuda -f Dockerfile_x64_cuda --build-arg "CUDA_ARCH=${CUDA_ARCH}" .
    else
        echo "----> Creating x64 image without cuda"
        build_x64
    fi

elif [[ $image_type == "l4t" ]]; then
    echo "----> Creating l4t image"
    docker build --network host -t dddnav:l4t_r36 -f Dockerfile_x64_l4t_r36 .

elif [[ $image_type == "x64_gz" ]]; then
    echo "----> Creating x64 base image, then Gazebo simulation layer"
    build_x64
    echo "----> Starting second layer with Gazebo (dddnav_gz:x64)"
    docker build --network host -t dddnav_gz:x64 -f Dockerfile_x64_gazebo . --no-cache

else
    echo "Invalid image type. Please choose: x64 | l4t | x64_gz"
    exit 1
fi
