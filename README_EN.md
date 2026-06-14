# dddnav_navigation

3D mapping / localization / navigation stack for wheeled and legged platforms running Livox Mid360 + IMU, with optional RealSense + DDRNet semantic segmentation.

Forked and reworked from [dddmr_navigation](https://github.com/dfl-rlab/dddmr_navigation) (BSD-3-Clause). Key differences vs upstream:

- SLAM swapped to **FAST-LIO2 (front-end, 100 Hz)** + **LIO-SAM (back-end loop closure)**
- Added **`dddnav_pose_fusion`** — SE(3) ESKF fusing FAST-LIO and MCL 3DL into 100 Hz `/odom_filtered` and ownership of `map → odom`; adaptive Q + Mahalanobis gate + degraded-state machine
- **MCL 3DL adaptive particle count** + added **`dddnav_utils/std_global_init`** (STD triangle-descriptor global re-localization with online watchdog)
- Bringup uses `common_nodes.py` for shared blocks; navigation tuning flattened to `base + overlay`
- Boot-time preflight (`dddnav_preflight`) + runtime telemetry (`slam_health_monitor` / `nav_perf_monitor`) all on `/diagnostics`

Please keep upstream attribution and the BSD-3-Clause licence on redistribution. 中文版: [README.md](README.md).

---

## Install dependencies

ROS 2 Humble + system packages:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop ros-humble-pcl-ros ros-humble-pcl-conversions \
  ros-humble-image-geometry ros-humble-cv-bridge \
  ros-humble-tf2-eigen ros-humble-tf2-geometry-msgs \
  ros-humble-message-filters ros-humble-diagnostic-msgs ros-humble-diagnostic-updater \
  libpcl-dev libceres-dev libeigen3-dev \
  python3-colcon-common-extensions python3-rosdep
```

Third-party deps (FAST-LIO / LIO-SAM / Livox driver / mcl) via rosdep:

```bash
sudo rosdep init    # first time only
rosdep update
cd /path/to/dddnav_navigation
rosdep install --from-paths src --ignore-src -y \
  --skip-keys "livox_ros_driver2"   # driver needs Livox SDK, install per upstream
```

Optional: CUDA + TensorRT (semantic segmentation / YOLO TRT inference).
Refer to `dddnav_docker/Dockerfile_x64_cuda` / `Dockerfile_l4t_cuda` for tested
combinations, or use the pre-built images.

---

## Build

From the workspace root (the directory containing `src/`):

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Pre-built Docker images are available under `dddnav_docker/`; see [`dddnav_docker/README.md`](dddnav_docker/README.md).

---

## Launch

Every entry launch accepts `nav_profile:=<name>` to switch tuning profiles (see *Configuration* below).

### Mapping

```bash
ros2 launch dddnav_bringup mapping.launch.py                       # LiDAR only
ros2 launch dddnav_bringup mapping_with_camera.launch.py           # + RealSense + DDRNet
ros2 launch dddnav_bringup mapping_nav.launch.py                   # mapping + planner
ros2 launch dddnav_bringup mapping_nav_with_camera.launch.py
```

Save the map automatically:

```bash
ros2 launch dddnav_bringup mapping.launch.py auto_save_on_exit:=true
# Ctrl-C triggers dddnav_utils/save_map_on_exit.py, which calls
#   /save_liosam_posegraph -> /lio_sam/save_map
# in order. Output lands in share/dddnav_bringup/map/
# (including lio_sam/std_db.bin and poses.pcd).
```

Or save manually (order matters — pose graph first, then LIO-SAM):

```bash
ros2 service call /save_liosam_posegraph std_srvs/srv/Empty {}
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2}"
```

### Localization + Navigation

Requires a saved map under `share/dddnav_bringup/map/`:

```bash
ros2 launch dddnav_bringup localization.launch.py                  # LiDAR only
ros2 launch dddnav_bringup localization_with_camera.launch.py      # + semantic point cloud
```

`std_global_init` queries the saved STD (Stable Triangle Descriptor) database on the first scans and publishes `/initial_3d_pose` once it has a confident match — no need to click an initial pose in RViz. If `std_db.bin` is missing the node exits silently and `runtime.yaml.initial_pose` is used as a fallback.

---

## Configuration

Tuning rule of thumb: **edit yaml, not launch.py**.

| File | Purpose |
|------|---------|
| [`dddnav_bringup/config/reality/runtime.yaml`](src/dddnav_bringup/config/reality/runtime.yaml) | LiDAR / camera mounts, startup delays, initial pose, driver rate |
| [`dddnav_bringup/config/reality/keyframes_mid360.yaml`](src/dddnav_bringup/config/reality/keyframes_mid360.yaml) | Keyframe extraction thresholds |
| [`dddnav_bringup/config/nav_base.yaml`](src/dddnav_bringup/config/nav_base.yaml) | Shared nav defaults (robot footprint, controller frequency, planner graph) |
| [`dddnav_bringup/config/reality/nav/mid360_*.yaml`](src/dddnav_bringup/config/reality/nav/) | Mode overlays: `mid360_mapping[_with_camera]` / `mid360_localization[_with_camera/_with_depth_camera]` |
| [`dddnav_bringup/config/reality/slam/fastlio_mid360.yaml`](src/dddnav_bringup/config/reality/slam/fastlio_mid360.yaml) | FAST-LIO front-end tuning (IMU noise, extrinsics, voxel) |
| [`dddnav_bringup/config/reality/slam/liosam_mid360.yaml`](src/dddnav_bringup/config/reality/slam/liosam_mid360.yaml) | LIO-SAM back-end tuning (IMU, loop closure, STD database) |
| [`dddnav_bringup/config/reality/pose_fusion.yaml`](src/dddnav_bringup/config/reality/pose_fusion.yaml) | ESKF Q/R, ZUPT, adaptive Q, auto-init |

Profile switch:

```bash
ros2 launch dddnav_bringup localization.launch.py nav_profile:=mid360_localization_with_depth_camera
ros2 launch dddnav_bringup localization.launch.py nav_profile:=/abs/path/custom.yaml
```

To create a new profile, copy `nav/mid360_localization.yaml` and edit only the keys you need to override.

Per-scenario tuning index: [`config/PARAMETERS.md`](src/dddnav_bringup/config/PARAMETERS.md).

---

## Packages

### SLAM / localization / navigation

| Package | Role |
|---------|------|
| [`dddnav_bringup`](src/dddnav_bringup/) | Single entry launches, navigation tuning yaml, TF / startup-delay configuration |
| [`FAST_LIO`](src/FAST_LIO/) | LiDAR-Inertial front-end. Publishes 100 Hz `/Odometry` and `/cloud_registered_body` |
| [`LIO-SAM`](src/LIO-SAM/) | Factor-graph back-end with three-stage loop closure (External → Scan Context → distance) |
| [`dddnav_mcl_3dl`](src/dddnav_mcl_3dl/) | 3D Monte-Carlo localization on the saved pose graph |
| [`dddnav_mcl_feature`](src/dddnav_mcl_feature/) | Edge / surface / ground feature extraction for MCL |
| [`dddnav_pose_fusion`](src/dddnav_pose_fusion/) | SE(3) ESKF fusing FAST-LIO and MCL → `/odom_filtered` and `map → odom` |
| [`dddnav_global_planner`](src/dddnav_global_planner/) | A* on a 3D ground graph |
| [`dddnav_local_planner`](src/dddnav_local_planner/) | DWA-like sampling planner with cuboid collision and critic scoring; trajectory_generators / mpc_critics / recovery_behaviors (incl. optional MPPI) |
| [`dddnav_p2p_move_base`](src/dddnav_p2p_move_base/) | move_base node and finite-state machine |
| [`dddnav_perception_3d`](src/dddnav_perception_3d/) | 3D costmap pipeline; plugins: `StaticLayer`, `MultiLayerSpinningLidar`, `DepthCameraLayer` |

### Sensing / vision

| Package | Role |
|---------|------|
| [`livox_ros_driver2`](src/livox_ros_driver2/) | Livox Mid360 ROS 2 driver |
| [`dddnav_semantic_segmentation`](src/dddnav_semantic_segmentation/) | DDRNet + TensorRT, publishes `/sematic_segmentation_point_cloud` |
| [`dddnav_yolo_trt`](src/dddnav_yolo_trt/) | YOLOv8 + TensorRT (optional, build with `-DTRT_ENABLED=ON`) |

### Utilities / messages / visualisation

| Package | Role |
|---------|------|
| [`dddnav_utils`](src/dddnav_utils/) | Livox→LIO-SAM bridge (C++), `liosam_to_posegraph`, `std_global_init`, `dddnav_preflight`, `slam_health_monitor`, `nav_perf_monitor`, `save_map_on_exit` |
| [`dddnav_sys_core`](src/dddnav_sys_core/) | Shared types and service definitions |
| [`dddnav_cloud_msgs`](src/dddnav_cloud_msgs/) | Custom point-cloud messages used by `dddnav_mcl_feature` |
| [`dddnav_std_descriptor`](src/dddnav_std_descriptor/) | STD triangle descriptor (mapping-time DB / localization-time retrieval) |
| [`dddnav_rviz_tools`](src/dddnav_rviz_tools/) | RViz panel plugins |
| [`dddnav_odom_3d`](src/dddnav_odom_3d/) | 3D odometry example |

---

## Data flow (topic chain)

**Mapping chain**

```
livox_ros_driver2 ──/livox/lidar (BE)──► livox_pc2_to_liosam ──/livox/lidar_liosam (R)─────► LIO-SAM (imageProjection→FA→IMU→mapOpt)
                                                            └──/livox/lidar_liosam_xyzi──► FAST-LIO ──/Odometry (100 Hz, owns odom→base TF)
                                                                                                  └──/cloud_registered_body
LIO-SAM mapOpt ──/lio_sam/mapping/odometry──► (owns map→odom TF, mapping mode only)
LIO-SAM keyframes ──► liosam_to_posegraph ─► dddnav_bringup/map/  (poses.pcd / pcd/N_*.pcd / lio_sam/std_db.bin)
```

`(BE)` = BEST_EFFORT, `(R)` = RELIABLE. Mismatched QoS across this bridge is a common silent-failure source — see the "Bridge QoS" section in [`dddnav_utils` README](src/dddnav_utils/README.md).

**Localization chain**

```
                                ┌─ /Odometry (100 Hz) ──────────────► pose_fusion (predict)
LiDAR ──► (same as mapping) ────┤                                        │
                                ├─ /laser_cloud_{sharp,flat,...} ──► mcl_3dl ──/mcl_pose──► pose_fusion (update)
                                │                                                ▲
                                └─ keyframe cloud ──► std_global_init ──/initial_3d_pose──► mcl_3dl
                                                          ▲
                                  /odom_filtered ─────────┘ (online watchdog)

pose_fusion ──/odom_filtered (100 Hz)──► local_planner / nav_perf_monitor
            └─ TF map→odom

p2p_move_base ──action──► global_planner ──/path──► local_planner ──/cmd_vel──► chassis
                            ▲                            ▲
                            │ ground graph               │ local costmap + cuboid collision
                            └── perception_3d_global    └── perception_3d_local
                                  ▲
                                  ├─ StaticLayer (pose graph)
                                  ├─ MultiLayerSpinningLidar (live cloud)
                                  └─ DepthCameraLayer (semantic cloud, optional)
```

**Camera branch** (`*_with_camera`) adds:

```
RealSense RGB ──► ddrnet_ros_img_sub.py ──/mask──┐
RealSense depth ─────────────────────────────────┴► semantic_segmentation2point_cloud
                                                     └──/sematic_segmentation_point_cloud──► perception_3d::DepthCameraLayer
```

**Telemetry & preflight** (always on)

```
all nodes ──► dddnav_preflight (boot-time QoS / TF / cross-node param checks)
          ──► slam_health_monitor (odom liveness / TF / cov)
          ──► nav_perf_monitor (cmd_vel / planner rate + FAST-LIO residuals)
          ──► pose_fusion ──/localization_status (mode / quality / age)
                          ▼
                    /diagnostics ──► Foxglove / RViz Diagnostic panel
```

---

## TF chain ownership

| Edge | Owner | Rate |
|------|-------|------|
| `base_link` → `livox_frame` | `static_transform_publisher` | static |
| `odom` → `base_link` | FAST-LIO | 100 Hz |
| `map` → `odom` | mapping mode: LIO-SAM `mapOptimization` / localization mode: `pose_fusion` | — |

MCL runs with `publish_tf=false`; LIO-SAM uses `publish_tf=false` in localization mode. Always confirm the owner before changing — having two publishers on the same edge will jitter `tf2` lookups. `dddnav_preflight` enforces this at boot.

---

## Runtime monitoring

The localization stack starts two watchdogs in addition to the main nodes; both publish to `/diagnostics`:

| Node | Watches |
|------|---------|
| `slam_health_monitor.py` | `/Odometry` / LIO-SAM odom rate, TF edge liveness, `map → odom` jumps, `/odom_filtered` covariance trace |
| `nav_perf_monitor.py` | `/Odometry` / `/odom_filtered` / `cmd_vel` / `/global_planner/path` rate + age, FAST-LIO residual and effective feature count |

Subscribe to `/diagnostics` from Foxglove or the RViz Diagnostic panel for live status.

---

## Other docs

| Topic | Link |
|-------|------|
| Bringup / map / camera | [src/dddnav_bringup/README.md](src/dddnav_bringup/README.md) |
| Docker images | [dddnav_docker/README.md](dddnav_docker/README.md) |
| Pose fusion (ESKF + adaptive Q) | [src/dddnav_pose_fusion/README.md](src/dddnav_pose_fusion/README.md) |
| Utils / STD global init / preflight / health | [src/dddnav_utils/README.md](src/dddnav_utils/README.md) |
| STD descriptor (place recognition) | [src/dddnav_std_descriptor/README.md](src/dddnav_std_descriptor/README.md) |
| MCL 3DL | [src/dddnav_mcl_3dl/README.md](src/dddnav_mcl_3dl/README.md) |
| MCL features | [src/dddnav_mcl_feature/README.md](src/dddnav_mcl_feature/README.md) · [src/dddnav_cloud_msgs/README.md](src/dddnav_cloud_msgs/README.md) |
| Perception 3D | [src/dddnav_perception_3d/README.md](src/dddnav_perception_3d/README.md) |
| Global / local planner | [src/dddnav_global_planner/README.md](src/dddnav_global_planner/README.md) · [src/dddnav_local_planner/README.md](src/dddnav_local_planner/README.md) |
| P2P move_base | [src/dddnav_p2p_move_base/README.md](src/dddnav_p2p_move_base/README.md) |
| Semantic / TRT | [src/dddnav_semantic_segmentation/README.md](src/dddnav_semantic_segmentation/README.md) · [src/dddnav_yolo_trt/README.md](src/dddnav_yolo_trt/README.md) |
| sys_core / rviz_tools | [src/dddnav_sys_core/README.md](src/dddnav_sys_core/README.md) · [src/dddnav_rviz_tools/README.md](src/dddnav_rviz_tools/README.md) |

---

## License

BSD-3-Clause (inherited from upstream dddmr_navigation). When redistributing, keep [`LICENSE`](LICENSE) and the upstream attribution intact.
