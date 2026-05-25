# dddnav_navigation

3D 建图 / 定位 / 导航栈，主要面向 Livox Mid360 + IMU + 可选 RealSense + DDRNet 的轮式 / 足式底盘。

基于 [dddmr_navigation](https://github.com/dfl-rlab/dddmr_navigation) (BSD-3-Clause) 二次开发，主要变动：

- SLAM 路线换成 **FAST-LIO2 (前端 100Hz)** + **LIO-SAM (后端回环, Scan Context + GICP)**，不再用 LeGO-LOAM
- 加 **`dddnav_pose_fusion`**：SE(3) ESKF 融合 FAST-LIO 与 MCL 3DL，输出 100Hz `/odom_filtered` 和 `map→odom`；自适应 Mahalanobis 门 + MCL 协方差硬拒，鲁棒性显著提升
- **MCL 3DL 自适应粒子数**：重定位 / 低 match_ratio 时按需放大（KLD 风格的轻量替代），收敛后指数回落
- 加 **`dddnav_utils/sc_global_init`**：启动期 Scan Context 全局重定位 + 运行期在线 watchdog（被搬运 / 走错楼层 / MCL 卡死自动恢复，spatial gate 杀掉重复几何误匹配）
- bringup launch 重构（`common_nodes.py` 抽公共组件）+ nav 调参 yaml `base + overlay` 化（`config/nav/`）
- Docker 镜像：x64 / CUDA / Jetson L4T / Gazebo 四档（[`dddnav_docker/`](dddnav_docker/)）
- 运行时 telemetry：`slam_health_monitor` + `nav_perf_monitor` 都发到 `/diagnostics`

发布或转发请保留上游致谢与 BSD-3-Clause 许可证。English: [README_EN.md](README_EN.md)。

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
# 输出落到 share/dddnav_bringup/map/  (含 lio_sam/sc_db.bin、poses.pcd 等)
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

启动时 `sc_global_init` 自动用 Scan Context 拉初始位姿，**无需手动点 RViz**。无 sc_db.bin 时回退到 `runtime.yaml.initial_pose`，仍可手动发 `/initial_3d_pose`。

---

## 配置文件

调参原则：**不改 launch.py，只改 yaml**。

| 文件 | 作用 |
|------|------|
| [`dddnav_bringup/config/runtime.yaml`](src/dddnav_bringup/config/runtime.yaml) | LiDAR / 相机外参，启动延时，初始位姿，驱动频率 |
| [`dddnav_bringup/config/keyframes_mid360.yaml`](src/dddnav_bringup/config/keyframes_mid360.yaml) | 关键帧抽取阈值（`keyframe_dist` / `keyframe_angle`） |
| [`dddnav_bringup/config/nav/base.yaml`](src/dddnav_bringup/config/nav/base.yaml) | 公共导航参数（机器人外形、控制频率、规划器） |
| [`dddnav_bringup/config/nav/mid360_*.yaml`](src/dddnav_bringup/config/nav/) | 模式 overlay：`mid360_mapping[_with_camera]` / `mid360_localization[_with_camera/_with_depth_camera]` |
| [`LIO-SAM/config/params_mid360.yaml`](src/LIO-SAM/config/params_mid360.yaml) | LIO-SAM 全部调参（IMU、回环、Scan Context） |
| [`dddnav_pose_fusion/config/pose_fusion.yaml`](src/dddnav_pose_fusion/config/pose_fusion.yaml) | ESKF Q/R、ZUPT、自适应 Q、auto-init |

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
| [`dddnav_utils`](src/dddnav_utils/) | Livox→LIO-SAM 桥接 (cpp), `liosam_to_posegraph`, `sc_global_init`, `slam_health_monitor`, `nav_perf_monitor`, `save_map_on_exit` |
| [`dddnav_sys_core`](src/dddnav_sys_core/) | 共享类型与 service 定义 |
| [`cloud_msgs`](src/cloud_msgs/) | mcl_feature 用的自定义点云消息 |
| [`dddnav_rviz_tools`](src/dddnav_rviz_tools/) | RViz 面板插件 |
| [`dddnav_odom_3d`](src/dddnav_odom_3d/) | 3D 里程计示例 |
| [`gz_quadbot`](src/gz_quadbot/) | Go2 Gazebo 仿真 |

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
| Utils / SC global init / health | [src/dddnav_utils/README.md](src/dddnav_utils/README.md) |
| MCL 3DL | [src/dddnav_mcl_3dl/README.md](src/dddnav_mcl_3dl/README.md) |
| MCL features | [src/dddnav_mcl_feature/README.md](src/dddnav_mcl_feature/README.md) |
| Perception 3D | [src/dddnav_perception_3d/README.md](src/dddnav_perception_3d/README.md) |
| Global / Local planner | [src/dddnav_global_planner/README.md](src/dddnav_global_planner/README.md) · [src/dddnav_local_planner/README.md](src/dddnav_local_planner/README.md) |
| P2P move_base | [src/dddnav_p2p_move_base/README.md](src/dddnav_p2p_move_base/README.md) |
| Semantic / TRT | [src/dddnav_semantic_segmentation/README.md](src/dddnav_semantic_segmentation/README.md) · [src/dddnav_yolo_trt/README.md](src/dddnav_yolo_trt/README.md) |
| sys_core / rviz_tools | [src/dddnav_sys_core/README.md](src/dddnav_sys_core/README.md) · [src/dddnav_rviz_tools/README.md](src/dddnav_rviz_tools/README.md) |
| Gazebo Go2 | [src/gz_quadbot/README.md](src/gz_quadbot/README.md) |

---

## License

BSD-3-Clause（继承自 dddmr_navigation 上游）。重新分发请保留 [`LICENSE`](LICENSE) 文件与上游致谢。
