# dddnav_navigation

3D mapping / localization / navigation stack for wheeled and legged platforms running Livox Mid360 + IMU, with optional RealSense + DDRNet semantic segmentation.

Forked and reworked from [dddmr_navigation](https://github.com/dfl-rlab/dddmr_navigation) (BSD-3-Clause). Notable changes:

- SLAM swapped to **FAST-LIO2 (front-end, 100 Hz)** + **LIO-SAM (back-end, Scan Context + GICP loop closure)**, replacing LeGO-LOAM
- Added **`dddnav_pose_fusion`** — SE(3) ESKF that fuses FAST-LIO and MCL 3DL into a 100 Hz `/odom_filtered` and owns `map → odom`
- Added **`dddnav_utils/sc_global_init`** — Scan Context global localiser at boot, no operator-supplied initial pose required
- Bringup launches refactored (`common_nodes.py` for shared blocks); navigation tuning flattened to `base + overlay` yaml under `config/nav/`
- Docker images: x64 / CUDA / Jetson L4T / Gazebo (see [`dddnav_docker/`](dddnav_docker/))
- Runtime telemetry: `slam_health_monitor` + `nav_perf_monitor`, both publishing to `/diagnostics`

Please keep upstream attribution and the BSD-3-Clause licence on redistribution. 中文版: [README.md](README.md).

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
# (including lio_sam/sc_db.bin and poses.pcd).
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

`sc_global_init` queries the saved Scan Context database on the first scans and publishes `/initial_3d_pose` once it has a confident match — no need to click an initial pose in RViz. If `sc_db.bin` is missing the node exits silently and `runtime.yaml.initial_pose` is used as a fallback.

---

## Configuration

Tuning rule of thumb: **edit yaml, not launch.py**.

| File | Purpose |
|------|---------|
| [`dddnav_bringup/config/runtime.yaml`](src/dddnav_bringup/config/runtime.yaml) | LiDAR / camera mounts, startup delays, initial pose, driver rate |
| [`dddnav_bringup/config/keyframes_mid360.yaml`](src/dddnav_bringup/config/keyframes_mid360.yaml) | Keyframe extraction thresholds |
| [`dddnav_bringup/config/nav/base.yaml`](src/dddnav_bringup/config/nav/base.yaml) | Shared nav defaults (robot footprint, controller frequency, planner graph) |
| [`dddnav_bringup/config/nav/mid360_*.yaml`](src/dddnav_bringup/config/nav/) | Mode overlays: `mid360_mapping[_with_camera]` / `mid360_localization[_with_camera/_with_depth_camera]` |
| [`LIO-SAM/config/params_mid360.yaml`](src/LIO-SAM/config/params_mid360.yaml) | LIO-SAM tuning (IMU, loop closure, Scan Context) |
| [`dddnav_pose_fusion/config/pose_fusion.yaml`](src/dddnav_pose_fusion/config/pose_fusion.yaml) | ESKF Q/R, ZUPT, adaptive Q, auto-init |

Profile switch:

```bash
ros2 launch dddnav_bringup localization.launch.py nav_profile:=mid360_localization_with_depth_camera
ros2 launch dddnav_bringup localization.launch.py nav_profile:=/abs/path/custom.yaml
```

To create a new profile, copy `nav/mid360_localization.yaml` and edit only the keys you need to override.

---

## Packages

### SLAM / localization / navigation

| Package | Role |
|---------|------|
| [`dddnav_bringup`](src/dddnav_bringup/) | Single entry launches, navigation tuning yaml, TF / startup-delay configuration |
| [`FAST_LIO`](src/FAST_LIO/) | LiDAR-Inertial front-end. Publishes 100 Hz `/Odometry` and `/cloud_registered_body` |
| [`LIO-SAM`](src/LIO-SAM/) | Factor-graph back-end with three-stage loop closure (External → Scan Context → distance) and SC database dump on save |
| [`dddnav_mcl_3dl`](src/dddnav_mcl_3dl/) | 3D Monte-Carlo localization on the saved pose graph |
| [`dddnav_mcl_feature`](src/dddnav_mcl_feature/) | Edge / surface / ground feature extraction for MCL |
| [`dddnav_pose_fusion`](src/dddnav_pose_fusion/) | SE(3) ESKF fusing FAST-LIO and MCL → `/odom_filtered` and `map → odom` |
| [`dddnav_global_planner`](src/dddnav_global_planner/) | A* on a 3D ground graph |
| [`dddnav_local_planner`](src/dddnav_local_planner/) | DWA-like sampling planner with cuboid collision and critic scoring; trajectory_generators / mpc_critics / recovery_behaviors |
| [`dddnav_p2p_move_base`](src/dddnav_p2p_move_base/) | move_base node and finite-state machine |
| [`dddnav_perception_3d`](src/dddnav_perception_3d/) | 3D costmap pipeline; plugins: `StaticLayer`, `MultiLayerSpinningLidar`, `DepthCameraLayer` |

### Sensing / vision

| Package | Role |
|---------|------|
| [`livox_ros_driver2`](src/livox_ros_driver2/) | Livox Mid360 ROS 2 driver |
| [`dddnav_semantic_segmentation`](src/dddnav_semantic_segmentation/) | DDRNet + TensorRT, publishes `/sematic_segmentation_point_cloud` |
| [`dddnav_trt`](src/dddnav_trt/) | YOLOv8 + TensorRT (optional, build with `-DTRT_ENABLED=ON`) |

### Utilities / messages / visualisation

| Package | Role |
|---------|------|
| [`dddnav_utils`](src/dddnav_utils/) | Livox→LIO-SAM bridge (C++), `liosam_to_posegraph`, `sc_global_init`, `slam_health_monitor`, `nav_perf_monitor`, `save_map_on_exit` |
| [`dddnav_sys_core`](src/dddnav_sys_core/) | Shared types and service definitions |
| [`cloud_msgs`](src/cloud_msgs/) | Custom point-cloud messages used by `dddnav_mcl_feature` |
| [`dddnav_rviz_tools`](src/dddnav_rviz_tools/) | RViz panel plugins |
| [`dddnav_odom_3d`](src/dddnav_odom_3d/) | 3D odometry example |
| [`gz_quadbot`](src/gz_quadbot/) | Go2 Gazebo simulation |

---

## TF chain ownership

| Edge | Owner | Rate |
|------|-------|------|
| `base_link` → `livox_frame` | `static_transform_publisher` | static |
| `odom` → `base_link` | FAST-LIO | 100 Hz |
| `map` → `odom` | mapping mode: LIO-SAM `mapOptimization` / localization mode: `pose_fusion` | — |

MCL runs with `publish_tf=false`; LIO-SAM uses `publish_tf=false` in localization mode. Always confirm the owner before changing — having two publishers on the same edge will jitter `tf2` lookups.

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
| Utils / SC global init / health | [src/dddnav_utils/README.md](src/dddnav_utils/README.md) |
| MCL 3DL | [src/dddnav_mcl_3dl/README.md](src/dddnav_mcl_3dl/README.md) |
| MCL features | [src/dddnav_mcl_feature/README.md](src/dddnav_mcl_feature/README.md) |
| Perception 3D | [src/dddnav_perception_3d/README.md](src/dddnav_perception_3d/README.md) |
| Global / local planner | [src/dddnav_global_planner/README.md](src/dddnav_global_planner/README.md) · [src/dddnav_local_planner/README.md](src/dddnav_local_planner/README.md) |
| P2P move_base | [src/dddnav_p2p_move_base/README.md](src/dddnav_p2p_move_base/README.md) |
| Semantic / TRT | [src/dddnav_semantic_segmentation/README.md](src/dddnav_semantic_segmentation/README.md) · [src/dddnav_trt/README.md](src/dddnav_trt/README.md) |
| sys_core / rviz_tools | [src/dddnav_sys_core/README.md](src/dddnav_sys_core/README.md) · [src/dddnav_rviz_tools/README.md](src/dddnav_rviz_tools/README.md) |
| Gazebo Go2 | [src/gz_quadbot/README.md](src/gz_quadbot/README.md) |

---

## License

BSD-3-Clause (inherited from upstream dddmr_navigation). When redistributing, keep [`LICENSE`](LICENSE) and the upstream attribution intact.
