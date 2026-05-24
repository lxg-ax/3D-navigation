# dddnav_navigation

3D mapping / localization / planning stack (multi-floor, 3D costmaps, etc.), beyond what [Nav2](https://github.com/ros-navigation/navigation2) ships by default. Based on [dddmr_navigation](https://github.com/dfl-rlab/dddmr_navigation) (BSD-3-Clause — keep attribution if you redistribute). 说明：在上游工程上改了 SLAM 路线、bringup、Docker 等；发布时请保留致谢与许可证要求。

**Paths:** `colcon build` from the repo root (directory that contains `src/`). Docker scripts mount that tree at **`/root/dddnav_navigation`** inside the container; on the host use `src/...`, inside the container use `/root/dddnav_navigation/...`.

No demo GIFs embedded here (optional: add under `docs/`). Upstream showcase media stays with [dddmr_navigation](https://github.com/dfl-rlab/dddmr_navigation).

---

## Default SLAM (Mid360)

**FAST-LIO2** (`fast_lio`) + **LIO-SAM** (`lio_sam`, Scan Context + GICP; loop closure: `loopClosureEnableFlag` in `src/LIO-SAM/config/params_mid360.yaml`). Typical FAST-LIO topics: `/Odometry`, `/cloud_registered`.

Optional **YOLOv8 + TensorRT**: [`dddnav_trt`](src/dddnav_trt/), build with `-DTRT_ENABLED=ON`。具体训练权重与雷达安装角以包内 CMake/代码为准。

---

## Go2 in Gazebo

[`src/gz_quadbot/`](src/gz_quadbot/). `dddnav_gz:x64` may clone the same upstream into `/ws_gz` — pick **either** vendored `src/gz_quadbot` **or** that image workflow unless you know you need both. Details: [dddnav_docker/README.md](dddnav_docker/README.md).

---

## Packages (folder → role)

| Path | Role |
|------|------|
| [dddnav_bringup](src/dddnav_bringup/) | Launches: mapping / mapping+nav / localization (+ optional camera stack)。**导航调参 yaml 集中在 [`dddnav_bringup/config/nav/`](src/dddnav_bringup/config/nav/)**，launch 用 `nav_profile:=<name>` 切换 |
| [dddnav_global_planner](src/dddnav_global_planner/) | 3D global planning |
| [dddnav_local_planner](src/dddnav_local_planner/) | `local_planner`, `mpc_critics`, `trajectory_generators`, `recovery_behaviors`, `base_trajectory` |
| [dddnav_p2p_move_base](src/dddnav_p2p_move_base/) | `p2p_move_base` 节点（Go2 yaml 仍在本包 `config/`，Mid360 主线 yaml 已搬到 `dddnav_bringup/config/nav/`） |
| [dddnav_sys_core](src/dddnav_sys_core/) | Shared types / services |
| [FAST_LIO](src/FAST_LIO/) | `fast_lio` |
| [LIO-SAM](src/LIO-SAM/) | `lio_sam` |
| [dddnav_mcl_3dl](src/dddnav_mcl_3dl/) | `mcl_3dl` |
| [dddnav_mcl_feature](src/dddnav_mcl_feature/) | MCL features |
| [dddnav_odom_3d](src/dddnav_odom_3d/) | 3D odom example |
| [dddnav_perception_3d](src/dddnav_perception_3d/) | ROS name **`perception_3d`** |
| [dddnav_pose_fusion](src/dddnav_pose_fusion/) | SE(3) ESKF: FAST-LIO 100Hz 预测 + MCL 3DL ~5Hz 量测 → `/odom_filtered` + `map→odom` |
| [dddnav_semantic_segmentation](src/dddnav_semantic_segmentation/) | DDRNet + TRT → semantic cloud |
| [dddnav_trt](src/dddnav_trt/) | YOLO TRT (optional) |
| [livox_ros_driver2](src/livox_ros_driver2/) | Livox driver |
| [dddnav_utils](src/dddnav_utils/) | Glue: Livox bridge (C++), pose-graph extractor, SLAM health monitor |
| [dddnav_rviz_tools](src/dddnav_rviz_tools/) | RViz panels |
| [gz_quadbot](src/gz_quadbot/) | Go2 Gazebo |
| [cloud_msgs](src/cloud_msgs/) | Custom point-cloud message definitions for `dddnav_mcl_feature` |

`dddnav_bringup` lists `lego_loam_bor` for older / Go2 demos; **default Mid360 bringup** uses `fast_lio` + `lio_sam`, not Lego LOAM. Check names with `ros2 pkg list` after build.

---

## Docker

| Image | Notes |
|-------|--------|
| `dddnav:x64` | Ubuntu 22.04, Humble, PCL 1.15, GTSAM 4.2a9 |
| `dddnav:cuda` | On top of x64: CUDA 12.6, cuDNN 9.6, TensorRT 10.7, PyTorch 2.8 |
| `dddnav:l4t_r36` | JetPack r36.4.0 base |
| `dddnav_gz:x64` | Gazebo layer |

```bash
cd /path/to/REPO/dddnav_docker/docker_file
./build.bash
./run_x64_gpu.bash   # or ./run_x64.bash
# in container:
cd /root/dddnav_navigation && source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Host `~/dddnav_bags` → container `/root/dddnav_bags` when using default run scripts. Full detail: [dddnav_docker/README.md](dddnav_docker/README.md).

---

## Semantic (DDRNet + TRT)

Live RealSense defaults **848×480** (`rs_semantic_segmentaton_trt_launch.py`); engine input **424×848**. Example bag launches read **`~/dddnav_bags/...`** (names like `rs435_rgbd_848x380` reflect how that bag was recorded).

```bash
# 仓库根：先转引擎，再 source 再 launch
cd src/dddnav_semantic_segmentation/model
# trtexec 路径随安装而变；NVIDIA 容器里常见 /usr/src/tensorrt/bin/trtexec
/usr/src/tensorrt/bin/trtexec \
  --onnx=ddrnet_23_slim_dualresnet_citys_best_model_424x848.onnx \
  --saveEngine=ddrnet_23_slim_dualresnet_citys_best_model_424x848.trt
cd ../../..
source install/setup.bash
ros2 launch dddnav_semantic_segmentation rs_semantic_segmentaton_trt_launch.py
# ros2 launch dddnav_semantic_segmentation bag_exclude_ss_trt_launch.py
```

Class IDs: `src/dddnav_semantic_segmentation/data/colors_mapillary.csv`.

---

## Bringup (main entry)

```bash
ros2 launch dddnav_bringup mapping.launch.py
ros2 launch dddnav_bringup mapping_with_camera.launch.py

ros2 launch dddnav_bringup mapping_nav.launch.py
ros2 launch dddnav_bringup mapping_nav_with_camera.launch.py

ros2 launch dddnav_bringup localization.launch.py
ros2 launch dddnav_bringup localization_with_camera.launch.py
```

常用参数（每个 launch 都支持）：

```bash
# 切换导航调参 profile（去 dddnav_bringup/config/nav/ 找对应 yaml）
ros2 launch dddnav_bringup localization.launch.py nav_profile:=mid360_localization_with_camera
ros2 launch dddnav_bringup mapping_nav.launch.py  nav_profile:=mid360_mapping_with_camera
# 也可以直接传绝对路径
ros2 launch dddnav_bringup localization.launch.py nav_profile:=/abs/path/custom.yaml

# 建图模式：启动清旧图 + ctrl-C 自动覆盖到 dddnav_bringup/map/
ros2 launch dddnav_bringup mapping.launch.py auto_save_on_exit:=true
```

Pose graph / map output: **`dddnav_bringup/map/`** (`share/dddnav_bringup/map` after install). `localization*.launch.py` sets `sub_maps.pose_graph_dir` there; `mapping*.launch.py` sets LIO-SAM `savePCDDirectory` there. **`*.pcd` under `map/` is not tracked in git**—run mapping locally, then save maps as in [dddnav_bringup/README.md](src/dddnav_bringup/README.md) (先 `/save_liosam_posegraph` 再 `/lio_sam/save_map`，或加 `auto_save_on_exit:=true` 让 launch 自动覆盖)。

关键帧抽取阈值统一在 [`dddnav_bringup/config/keyframes_mid360.yaml`](src/dddnav_bringup/config/keyframes_mid360.yaml)（`keyframe_dist` / `keyframe_angle` / `ground_angle_thresh`），`mapping*.launch.py` 通过 `bringup_paths.keyframes_yaml()` + `keyframes_save_dir_overlay()` 加载并把 `save_dir` 强制定向到 `dddnav_bringup/map/`。换室内/户外场景调这个 yaml 即可，不用改 launch。

回环检测：LIO-SAM 后端按优先级 External → Scan Context → 距离搜索三段式，最终 GICP 精对齐。Scan Context 的关键帧排除窗口、余弦距离阈值、启用所需的最小数据库规模都开放给 [`params_mid360.yaml`](src/LIO-SAM/config/params_mid360.yaml)（`scExcludeRecent` / `scDistThreshold` / `scMinDatabase`）。GPM (`p2p_global_plan_manager`) 按 `global_plan_query_frequency` 默认 5 Hz 持续重规划，回环修正 `map→odom` 后路径会自然刷新，无需额外触发。

`mapping*.launch.py` 不再发布静态 `map→odom` 占位 TF（之前会和 LIO-SAM `mapOptimization` 的动态广播抢同一条边）。LIO-SAM 起来前 RViz 看不到 `map` 帧属正常，等 5–10s 后端起来即可。

### 全局重定位（kidnapped robot）

`localization*.launch.py` 默认会启动 `dddnav_utils/sc_global_init`：用建图阶段写入 `share/dddnav_bringup/map/lio_sam/sc_db.bin` 的 Scan Context 描述符 + `poses.pcd` 关键帧位姿，对第一帧 LiDAR 做 SC 查询；连续 N 帧匹配同一关键帧后发 `/initial_3d_pose`，MCL 直接收敛。无需操作员在 RViz 点初始位姿。

如果地图里没有 `sc_db.bin`（旧地图，或建图时关掉了 SC），节点会安静退出，runtime.yaml 的 `initial_pose` 兜底仍然生效。手动 `/initial_3d_pose` 仍然能再次拉粒子。

### 运行时 telemetry

`nav_perf_monitor.py` 跟 `slam_health_monitor.py` 一起跑，两者都把状态发到 `/diagnostics`：

| 监控对象 | 节点 |
|---------|------|
| `/Odometry`、`/odom_filtered` 速率 / age | `nav_perf_monitor` |
| `cmd_vel` 频率（控制环卡顿） | `nav_perf_monitor` |
| `/global_planner/path` 重规划间隔 | `nav_perf_monitor` |
| FAST-LIO 残差 / 有效 correspondences | `nav_perf_monitor` |
| TF 边 liveness、`map→odom` 跳变 | `slam_health_monitor` |
| `/odom_filtered` 协方差 trace | `slam_health_monitor` |

Foxglove / RViz Diagnostic 面板直接订 `/diagnostics` 就能可视化。

### 自适应 ESKF（FAST-LIO 残差驱动）

FAST-LIO 在 `/fast_lio/health` 上发 `[scan_to_map_residual_m, effective_feats]`。`pose_fusion` 订阅它，残差超过基线时把过程噪声 Q 放大（最高 16x），稀疏对应（feats 不够）时也强制提权，让 MCL 的全局观测在颠簸 / 动态 / 长走廊时拿到更大权重。关掉的话把 `pose_fusion.yaml` 里 `adaptive_q_gain` 设为 0。

### TF 链路所有权

| 边 | 所有者 | 频率 | 备注 |
|----|--------|------|------|
| `base_link` → `livox_frame` | `static_transform_publisher` | 1Hz static | 安装外参，编辑 `runtime.yaml` 的 `lidar_mount` |
| `odom` → `base_link` | FAST-LIO (`fastlio_mapping`) | 100Hz | LIO-SAM 设 `publish_tf: false`，MCL 设 `publish_odom_tf: false` |
| `map` → `odom` | 见模式 | — | mapping: LIO-SAM `mapOptimization`；localization: `pose_fusion`；MCL `publish_tf: false` |

任意一条边出现两个发布者会让 tf2 lookups 抖动 —— 修改前先确认 owner。

### 配置中心化

| 文件 | 作用 |
|------|------|
| [`dddnav_bringup/config/runtime.yaml`](src/dddnav_bringup/config/runtime.yaml) | LiDAR 安装外参 / 启动延时 / 初始位姿 / 驱动频率 |
| [`dddnav_bringup/config/keyframes_mid360.yaml`](src/dddnav_bringup/config/keyframes_mid360.yaml) | 关键帧抽取阈值 |
| [`dddnav_bringup/config/nav/`](src/dddnav_bringup/config/nav/) | **导航调参主战场**：`mid360_mapping.yaml` / `mid360_localization.yaml` / `*_with_camera.yaml` / `*_with_depth_camera.yaml`。launch 用 `nav_profile:=<name>` 切换。复制一份改名即可成新 profile |
| [`LIO-SAM/config/params_mid360.yaml`](src/LIO-SAM/config/params_mid360.yaml) | LIO-SAM 全部调参（IMU、回环、Scan Context） |
| [`dddnav_pose_fusion/config/pose_fusion.yaml`](src/dddnav_pose_fusion/config/pose_fusion.yaml) | ESKF Q/R 协方差、Mahalanobis 门 |

调参原则：**不改 launch.py，只改 yaml**。launch 文件只负责拓扑和启动顺序。

Camera stack: build TRT engine as above; optional TF edits in `dddnav_bringup/launch/common_camera_nodes.py`.

> 现状：相机分支（`*_with_camera.launch.py`，含 RealSense + DDRNet 语义点云接入 perception_3d 与导航）目前**未在真机实测**，仅做了编译/语法验证。已知点：FAST-LIO + MCL 3DL 的 LiDAR 主线工作正常；接入深度相机后的语义层叠加、动态层避障、坐标系/时间戳对齐请按需自行验证后再上线。

---

## Other READMEs

| Topic | Link |
|-------|------|
| Bringup / map / camera | [src/dddnav_bringup/README.md](src/dddnav_bringup/README.md) |
| Docker | [dddnav_docker/README.md](dddnav_docker/README.md) |
| MCL | [src/dddnav_mcl_3dl/README.md](src/dddnav_mcl_3dl/README.md) |
| MCL features | [src/dddnav_mcl_feature/README.md](src/dddnav_mcl_feature/README.md) |
| Pose fusion (ESKF) | [src/dddnav_pose_fusion/README.md](src/dddnav_pose_fusion/README.md) |
| Utils / SLAM health | [src/dddnav_utils/README.md](src/dddnav_utils/README.md) |
| Cloud msgs | [src/cloud_msgs/README.md](src/cloud_msgs/README.md) |
| Perception | [src/dddnav_perception_3d/README.md](src/dddnav_perception_3d/README.md) |
| Global planner | [src/dddnav_global_planner/README.md](src/dddnav_global_planner/README.md) |
| Local planner | [src/dddnav_local_planner/README.md](src/dddnav_local_planner/README.md) |
| P2P / Go2 launch | [src/dddnav_p2p_move_base/README.md](src/dddnav_p2p_move_base/README.md) |
| Semantic | [src/dddnav_semantic_segmentation/README.md](src/dddnav_semantic_segmentation/README.md) |
| TRT YOLO | [src/dddnav_trt/README.md](src/dddnav_trt/README.md) |
| Odom 3D | [src/dddnav_odom_3d/README.md](src/dddnav_odom_3d/README.md) |
| sys_core / rviz_tools | [src/dddnav_sys_core/README.md](src/dddnav_sys_core/README.md) · [src/dddnav_rviz_tools/README.md](src/dddnav_rviz_tools/README.md) |
| Gazebo Go2 | [src/gz_quadbot/README.md](src/gz_quadbot/README.md) |
