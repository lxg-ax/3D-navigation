# dddnav_navigation

3D 建图 / 定位 / 导航栈，主要面向 Livox Mid360 + IMU + 可选 RealSense + DDRNet 的轮式 / 足式底盘。

基于 [dddmr_navigation](https://github.com/dfl-rlab/dddmr_navigation) (BSD-3-Clause) 二次开发。相对上游的主要差异：

- SLAM 路线换成 **FAST-LIO2 (前端 100Hz)** + **LIO-SAM (后端回环)**
- 加 **`dddnav_pose_fusion`**：SE(3) ESKF 融合 FAST-LIO 与 MCL 3DL，输出 100Hz `/odom_filtered` 与 `map→odom`，自适应 Q + Mahalanobis 门 + 退化降级
- **MCL 3DL 自适应粒子数** + 加 **`dddnav_utils/std_global_init`**（STD 三角描述子全局重定位 + 在线 watchdog）
- bringup `common_nodes.py` 抽公共组件，nav 调参 yaml 走 `base + overlay`
- 启动期自检 (`dddnav_preflight`) + 运行期 telemetry (`slam_health_monitor` / `nav_perf_monitor`) 全部走 `/diagnostics`

发布或转发请保留上游致谢与 BSD-3-Clause 许可证。English: [README_EN.md](README_EN.md)。

---

## 安装依赖

ROS 2 Humble + 系统包：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop ros-humble-pcl-ros ros-humble-pcl-conversions \
  ros-humble-image-geometry ros-humble-cv-bridge \
  ros-humble-tf2-eigen ros-humble-tf2-geometry-msgs \
  ros-humble-message-filters ros-humble-diagnostic-msgs ros-humble-diagnostic-updater \
  libpcl-dev libceres-dev libeigen3-dev libgeographic-dev \
  python3-colcon-common-extensions python3-rosdep
```

第三方依赖（FAST-LIO / LIO-SAM / Livox 驱动 / mcl 等）走 rosdep 拉齐：

```bash
sudo rosdep init    # 仅首次
rosdep update
cd /path/to/dddnav_navigation
rosdep install --from-paths src --ignore-src -y \
  --skip-keys "livox_ros_driver2"   # 驱动需要 Livox SDK，按下游说明手动装
```

可选：CUDA + TensorRT（语义分割 / YOLO 走 TRT 推理时需要）。版本组合参照
`dddnav_docker/Dockerfile_x64_cuda` / `Dockerfile_l4t_cuda`，或直接用现成镜像。

---

## 编译

仓库根（含 `src/`）下：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

也可以用 `dddnav_docker/` 里现成的镜像，详见 [`dddnav_docker/README.md`](dddnav_docker/README.md)。

---

## 启动

每个 launch 都支持 `nav_profile:=<name>` 切换调参，详见配置中心化一节。

### 建图

```bash
ros2 launch dddnav_bringup mapping.launch.py                       # 纯 LiDAR
ros2 launch dddnav_bringup mapping_with_camera.launch.py           # + RealSense + DDRNet
ros2 launch dddnav_bringup mapping_nav.launch.py                   # 边建边导航
ros2 launch dddnav_bringup mapping_nav_with_camera.launch.py
```

保存地图（自动）：

```bash
ros2 launch dddnav_bringup mapping.launch.py auto_save_on_exit:=true
# Ctrl-C 时由 dddnav_utils/save_map_on_exit.py 按顺序调
#   /save_liosam_posegraph  -> /lio_sam/save_map
# 输出落到 share/dddnav_bringup/map/  (含 lio_sam/std_db.bin、poses.pcd 等)
```

保存地图（手动，顺序不能反）：

```bash
ros2 service call /save_liosam_posegraph std_srvs/srv/Empty {}
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2}"
```

### 定位 + 导航

需先有 `share/dddnav_bringup/map/` 数据：

```bash
ros2 launch dddnav_bringup localization.launch.py                  # 纯 LiDAR
ros2 launch dddnav_bringup localization_with_camera.launch.py      # + 语义点云
```

启动时 `std_global_init` 自动用 STD（Stable Triangle Descriptor）三角描述子拉初始位姿，**无需手动点 RViz**。无 `std_db.bin` 时回退到 `runtime.yaml.initial_pose`，仍可手动发 `/initial_3d_pose`。

---

## 配置文件

调参原则：**不改 launch.py，只改 yaml**。

| 文件 | 作用 |
|------|------|
| [`dddnav_bringup/config/reality/runtime.yaml`](src/dddnav_bringup/config/reality/runtime.yaml) | LiDAR / 相机外参，启动延时，初始位姿，驱动频率 |
| [`dddnav_bringup/config/reality/keyframes_mid360.yaml`](src/dddnav_bringup/config/reality/keyframes_mid360.yaml) | 关键帧抽取阈值（`keyframe_dist` / `keyframe_angle`） |
| [`dddnav_bringup/config/nav_base.yaml`](src/dddnav_bringup/config/nav_base.yaml) | 公共导航参数（机器人外形、控制频率、规划器） |
| [`dddnav_bringup/config/reality/nav/mid360_*.yaml`](src/dddnav_bringup/config/reality/nav/) | 模式 overlay：`mid360_mapping[_with_camera]` / `mid360_localization[_with_camera/_with_depth_camera]` |
| [`LIO-SAM/config/params_mid360.yaml`](src/LIO-SAM/config/params_mid360.yaml) | LIO-SAM 全部调参（IMU、回环、Scan Context） |
| [`dddnav_bringup/config/reality/pose_fusion.yaml`](src/dddnav_bringup/config/reality/pose_fusion.yaml) | ESKF Q/R、ZUPT、自适应 Q、auto-init |

`nav_profile` 用法：

```bash
ros2 launch dddnav_bringup localization.launch.py nav_profile:=mid360_localization_with_depth_camera
ros2 launch dddnav_bringup localization.launch.py nav_profile:=/abs/path/custom.yaml
```

复制一份 `nav/mid360_localization.yaml` 改差异部分即可作为新 profile。

---

## 功能包

### 主线 SLAM / 定位 / 规划

| 包 | 作用 |
|------|------|
| [`dddnav_bringup`](src/dddnav_bringup/) | 一键启动入口，nav 调参 yaml，TF / 启动延时配置 |
| [`FAST_LIO`](src/FAST_LIO/) | 前端 LiDAR-Inertial 里程计，输出 100Hz `/Odometry` 和 `/cloud_registered_body` |
| [`LIO-SAM`](src/LIO-SAM/) | 后端因子图 + 回环（External / Scan Context / 距离搜索三段式）；保存时 dump SC 数据库供定位用 |
| [`dddnav_mcl_3dl`](src/dddnav_mcl_3dl/) | 3D 粒子滤波定位，订位姿图 + 当前点云 |
| [`dddnav_mcl_feature`](src/dddnav_mcl_feature/) | 给 MCL 的特征提取（边 / 面 / 地面） |
| [`dddnav_pose_fusion`](src/dddnav_pose_fusion/) | SE(3) ESKF：FAST-LIO 100Hz 预测 + MCL 5Hz 量测 → `/odom_filtered` + `map→odom` |
| [`dddnav_global_planner`](src/dddnav_global_planner/) | 3D ground-graph A* 全局规划 |
| [`dddnav_local_planner`](src/dddnav_local_planner/) | DWA + cuboid 碰撞 + critic 评分；含 trajectory_generators / mpc_critics / recovery_behaviors |
| [`dddnav_p2p_move_base`](src/dddnav_p2p_move_base/) | move_base 入口节点，FSM + cmd_vel 输出 |
| [`dddnav_perception_3d`](src/dddnav_perception_3d/) | 3D 代价地图，插件：`StaticLayer` / `MultiLayerSpinningLidar` / `DepthCameraLayer` 等 |

### 感知 / 视觉

| 包 | 作用 |
|------|------|
| [`livox_ros_driver2`](src/livox_ros_driver2/) | Livox Mid360 ROS 2 驱动 |
| [`dddnav_semantic_segmentation`](src/dddnav_semantic_segmentation/) | DDRNet + TensorRT 语义分割，输出 `/sematic_segmentation_point_cloud` |
| [`dddnav_yolo_trt`](src/dddnav_yolo_trt/) | YOLOv8 + TensorRT (可选, `-DTRT_ENABLED=ON`) |

### 工具 / 消息 / 可视化

| 包 | 作用 |
|------|------|
| [`dddnav_utils`](src/dddnav_utils/) | Livox→LIO-SAM 桥接 (cpp), `liosam_to_posegraph`, `std_global_init`, `dddnav_preflight`, `slam_health_monitor`, `nav_perf_monitor`, `save_map_on_exit` |
| [`dddnav_sys_core`](src/dddnav_sys_core/) | 共享类型与 service 定义 |
| [`dddnav_cloud_msgs`](src/dddnav_cloud_msgs/) | mcl_feature 用的自定义点云消息 |
| [`dddnav_std_descriptor`](src/dddnav_std_descriptor/) | STD 三角描述子（建图存 DB / 定位检索） |
| [`dddnav_rviz_tools`](src/dddnav_rviz_tools/) | RViz 面板插件 |
| [`dddnav_odom_3d`](src/dddnav_odom_3d/) | 3D 里程计示例 |

---

## 数据流（topic 链路）

**建图链**

```
livox_ros_driver2 ──/livox/lidar (BE)──► livox_pc2_to_liosam ──/livox/lidar_liosam (R)─────► LIO-SAM (imageProjection→FA→IMU→mapOpt)
                                                            └──/livox/lidar_liosam_xyzi──► FAST-LIO ──/Odometry (100Hz, 发 odom→base TF)
                                                                                                  └──/cloud_registered_body
LIO-SAM mapOpt ──/lio_sam/mapping/odometry──► (发 map→odom TF, 仅建图)
LIO-SAM 关键帧 ──► liosam_to_posegraph ─► dddnav_bringup/map/  (poses.pcd / pcd/N_*.pcd / lio_sam/std_db.bin)
```

`(BE)` = BEST_EFFORT，`(R)` = RELIABLE。桥接两端 QoS 不一致是常见隐性故障源，详见 [`dddnav_utils` README](src/dddnav_utils/README.md) 的 "Bridge QoS 兼容性" 段。

**定位链**

```
                                ┌─ /Odometry (100Hz) ───────────────► pose_fusion (predict)
LiDAR ──► (上行同建图) ──┤                                              │
                                ├─ /laser_cloud_{sharp,flat,...} ────► mcl_3dl ──/mcl_pose──► pose_fusion (update)
                                │                                                 ▲
                                └─ 关键帧点云 ──► std_global_init ──/initial_3d_pose──► mcl_3dl
                                                       ▲
                                  /odom_filtered ──────┘ (在线 watchdog 用)

pose_fusion ──/odom_filtered (100Hz)─► local_planner / nav_perf_monitor
            └─ TF map→odom

p2p_move_base ──action──► global_planner ──/path──► local_planner ──/cmd_vel──► 底盘
                            ▲                            ▲
                            │ ground graph               │ 局部代价图 + cuboid 碰撞
                            └── perception_3d_global    └── perception_3d_local
                                  ▲
                                  ├─ StaticLayer (位姿图)
                                  ├─ MultiLayerSpinningLidar (当前点云)
                                  └─ DepthCameraLayer (语义点云, 可选)
```

**带相机分支**（`*_with_camera`）补一支：

```
RealSense RGB ──► ddrnet_ros_img_sub.py ──/mask──┐
RealSense depth ─────────────────────────────────┴► semantic_segmentation2point_cloud
                                                     └──/sematic_segmentation_point_cloud──► perception_3d::DepthCameraLayer
```

**telemetry 与自检**（始终在跑）

```
所有节点 ──► dddnav_preflight (启动期 QoS / TF / 跨节点参数自检)
         ──► slam_health_monitor (odom liveness / TF / cov)
         ──► nav_perf_monitor (cmd_vel / planner 频率 + FAST-LIO 残差)
         ──► pose_fusion ──/localization_status (mode / quality / age)
                          ▼
                    /diagnostics ─► Foxglove / RViz Diagnostic 面板
```

---

## TF 链路所有权

| 边 | 所有者 | 频率 |
|----|--------|------|
| `base_link` → `livox_frame` | `static_transform_publisher` | static |
| `odom` → `base_link` | FAST-LIO | 100Hz |
| `map` → `odom` | mapping: LIO-SAM `mapOptimization` / localization: `pose_fusion` | — |

`MCL` 设 `publish_tf=false`，`LIO-SAM` 在 localization 流程里 `publish_tf=false`。修改前先确认所有者，避免一条边两个广播者。

---

## 运行时监控

定位流程会同时启动两个 watchdog，全部状态发 `/diagnostics`：

| 节点 | 监控 |
|------|------|
| `slam_health_monitor.py` | `/Odometry` / LIO-SAM odom 速率, TF 边 liveness, `map→odom` 跳变, `/odom_filtered` cov trace |
| `nav_perf_monitor.py` | `/Odometry` / `/odom_filtered` / `cmd_vel` / 全局规划路径 速率 + age, FAST-LIO 残差 + 有效特征数 |

Foxglove / RViz Diagnostic 面板订 `/diagnostics` 即可。

---

## 其他文档

| 主题 | 链接 |
|------|------|
| Bringup / 地图 / 相机 | [src/dddnav_bringup/README.md](src/dddnav_bringup/README.md) |
| Docker 镜像 | [dddnav_docker/README.md](dddnav_docker/README.md) |
| Pose fusion (ESKF + Adaptive Q) | [src/dddnav_pose_fusion/README.md](src/dddnav_pose_fusion/README.md) |
| Utils / STD global init / preflight / health | [src/dddnav_utils/README.md](src/dddnav_utils/README.md) |
| STD descriptor (place recognition) | [src/dddnav_std_descriptor/README.md](src/dddnav_std_descriptor/README.md) |
| MCL 3DL | [src/dddnav_mcl_3dl/README.md](src/dddnav_mcl_3dl/README.md) |
| MCL features | [src/dddnav_mcl_feature/README.md](src/dddnav_mcl_feature/README.md) · [src/dddnav_cloud_msgs/README.md](src/dddnav_cloud_msgs/README.md) |
| Perception 3D | [src/dddnav_perception_3d/README.md](src/dddnav_perception_3d/README.md) |
| Global / Local planner | [src/dddnav_global_planner/README.md](src/dddnav_global_planner/README.md) · [src/dddnav_local_planner/README.md](src/dddnav_local_planner/README.md) |
| P2P move_base | [src/dddnav_p2p_move_base/README.md](src/dddnav_p2p_move_base/README.md) |
| Semantic / TRT | [src/dddnav_semantic_segmentation/README.md](src/dddnav_semantic_segmentation/README.md) · [src/dddnav_yolo_trt/README.md](src/dddnav_yolo_trt/README.md) |
| sys_core / rviz_tools | [src/dddnav_sys_core/README.md](src/dddnav_sys_core/README.md) · [src/dddnav_rviz_tools/README.md](src/dddnav_rviz_tools/README.md) |

---

## License

BSD-3-Clause（继承自 dddmr_navigation 上游）。重新分发请保留 [`LICENSE`](LICENSE) 文件与上游致谢。
