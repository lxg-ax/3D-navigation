# dddnav_docker

Ubuntu 22.04 + ROS 2 Humble images for this workspace. Stack overview: [../README.md](../README.md).

## Images

| Tag | Dockerfile | Notes |
|-----|------------|--------|
| `dddnav:x64` | `docker_file/Dockerfile_x64` | Humble, PCL 1.15, GTSAM 4.2a9, colcon, [serial](https://github.com/wjwwood/serial) |
| `dddnav:cuda` | `docker_file/Dockerfile_x64_cuda` | On `dddnav:x64`: CUDA 12.6, cuDNN 9.6, TensorRT 10.7, PyTorch 2.8, OpenCV 4.11 (CUDA) |
| `dddnav:l4t_r36` | `docker_file/Dockerfile_x64_l4t_r36` | Base `nvcr.io/nvidia/l4t-jetpack:r36.4.0` |
| `dddnav_gz:x64` | `docker_file/Dockerfile_x64_gazebo` | Gazebo on `dddnav:x64`; clones [gz_quadbot](https://github.com/dfl-rlab/gz_quadbot) to `/ws_gz`. Repo also has `src/gz_quadbot/` — use one approach unless you need both. |

## Build

Repo root = directory with `src/` and `dddnav_docker/`.

```bash
cd /path/to/REPO/dddnav_docker/docker_file
chmod +x build.bash
./build.bash
```

Prompts: image `x64` | `l4t` | `x64_gz`; CUDA build Y/N (reads GPU arch from `nvidia-smi`). Context = `docker_file/`.

## Run

Mounts your repo at **`/root/dddnav_navigation`**.

```bash
cd /path/to/REPO/dddnav_docker/docker_file
./run_x64.bash
# or: ./run_x64_gpu.bash
```

Optional: host **`~/dddnav_bags`** → **`/root/dddnav_bags`** in container.

## In container

```bash
cd /root/dddnav_navigation
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

TRT YOLO package: add `-DTRT_ENABLED=ON` to `colcon` if you build `dddnav_trt` there.

## Registry

Typical tags: `dddnav:x64`, `dddnav:cuda`, `dddnav:l4t_r36`, `dddnav_gz:x64`. NVIDIA `.deb` URLs in Dockerfiles are pinned — update `wget` if links break.

## Misc

- `docker_file/rtps_udp_profile.xml` — Fast DDS profile, optional.
- `docker_file/bash/*.bash` — L4T helpers (PCL/GTSAM, librealsense, …).
