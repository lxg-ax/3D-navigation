# dddnav_bringup

一键启动：建图、边建图边导航、定位导航；每种有纯 LiDAR 与带 RealSense+DDRNet 版本。总览与路径约定：[根 README](../../README.md)。

**数据：** 位姿图等写到 **`map/`**（安装后 `share/dddnav_bringup/map`）。`localization*.launch.py` 为 `mcl_3dl` 注入 `sub_maps.pose_graph_dir`；`mapping*.launch.py` 为 LIO-SAM 注入 `savePCDDirectory`，均指向该目录。仓库**不提交** `map/` 下的 **`*.pcd`**（体积大、与环境相关）；克隆后需先**自行建图并保存**，再跑定位。

## 命令

**建图**
```bash
ros2 launch dddnav_bringup mapping.launch.py
ros2 launch dddnav_bringup mapping_with_camera.launch.py
```

保存（手动）：
```bash
ros2 service call /save_liosam_posegraph std_srvs/srv/Empty {}
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2}"
```
> 顺序：先 `/save_liosam_posegraph` 再 `/lio_sam/save_map`。LIO-SAM 的 save service 会 `rm -r` 自己的 `savePCDDirectory`，颠倒顺序会把 `liosam_to_posegraph` 已经写好的 `pcd/N_*.pcd` 删掉。

保存（自动覆盖）：
```bash
ros2 launch dddnav_bringup mapping.launch.py auto_save_on_exit:=true
# Ctrl-C 退出时自动调 dddnav_utils/save_map_on_exit.py，按顺序两个 service。
```
`auto_save_on_exit:=true` 时启动前会清空 `map/` 下的旧 pcd，并在退出时由 `save_map_on_exit.py`（rclpy）调用 `/save_liosam_posegraph` 与 `/lio_sam/save_map`，比从前的 `bash + ros2 service call` 链稳得多——launch tear-down 时 ROS 环境变量被改也不影响。LIO-SAM 的输出会落到 `map/lio_sam/` 子目录。`mapping_nav.launch.py` 同样支持这个参数。

**边建图边导航**
```bash
ros2 launch dddnav_bringup mapping_nav.launch.py
ros2 launch dddnav_bringup mapping_nav_with_camera.launch.py
```

**定位导航**（需先有 `map/` 数据）
```bash
ros2 launch dddnav_bringup localization.launch.py
ros2 launch dddnav_bringup localization_with_camera.launch.py
```

定位启动时不再需要手动给初始位姿：`sc_global_init` 会读 `map/lio_sam/sc_db.bin`（建图时 LIO-SAM `save_map` 自动写入）+ `map/poses.pcd`，对第一帧雷达做 SC 查表，连续 N 帧匹配同一关键帧就自动发 `/initial_3d_pose`。如果地图没 `sc_db.bin`，节点直接退出，仍由 `runtime.yaml.initial_pose` 兜底。

定位运行时还会启动两个监控：
* `slam_health_monitor.py`：TF 边 / odom 话题 liveness、`map→odom` 跳变、`/odom_filtered` cov trace
* `nav_perf_monitor.py`：`/Odometry` / `/odom_filtered` / `cmd_vel` / 全局规划路径速率 + age，FAST-LIO 残差 / feats

两者都把状态推到 `/diagnostics`，foxglove 直接订阅可视化。

## 带相机 (`*_with_camera`)

需要 RealSense（D435/D455）、DDRNet 的 TensorRT 引擎（在目标 GPU 上 `trtexec` 转 ONNX，见根 README「Semantic」）。相机安装外参在 [`config/runtime.yaml`](config/runtime.yaml) 的 `camera_mount` 下，与 `lidar_mount` 同级；改 yaml 即可，无需动 launch。

数据流：RGB/深度 → `ddrnet_ros_img_sub.py` → mask → `semantic_segmentation2point_cloud` → 话题 **`/sematic_segmentation_point_cloud`**（历史拼写）→ `perception_3d::DepthCameraLayer`。

| 模式 | Launch | 默认 nav profile |
|------|--------|---------------------------------------------|
| 建图+视觉 | `mapping_with_camera.launch.py` | — |
| 建图导航+视觉 | `mapping_nav_with_camera.launch.py` | `mid360_mapping_with_camera` |
| 定位+视觉 | `localization_with_camera.launch.py` | `mid360_localization_with_camera` |

## Launch 公共组件

`launch/common_nodes.py` 把每个入口 launch 共用的部件抽出来：

| Helper | 内容 |
|--------|------|
| `lidar_driver_and_tf(rt)` | Livox driver + `base_link→livox_frame` static TF |
| `lidar_front_end(rt, fastlio_yaml)` | livox→liosam 桥接 + FAST-LIO |
| `liosam_back_end(...)` | LIO-SAM 4 节点 + `liosam_to_posegraph` + `slam_health_monitor` |
| `localization_stack(...)` | MCL 3DL + pose_fusion + mcl_feature + 初始位姿 |
| `nav_stack(rt, params)` | global_planner + p2p_move_base + clicked2goal |
| `auto_save_actions(...)` | 启动前清旧图 + Ctrl-C 触发 `save_map_on_exit.py` |
| `rviz_action(rt, rviz_yaml)` | RViz 延时启动 |

每个入口 launch 现在都只是这些 helper 的拼装；要改某个节点的拓扑，到 `common_nodes.py` 改一处即可。

## 目录

`launch/` · `config/`（含 `config/nav/` 导航调参）· `rviz/`（mapping / mapping_nav / localization）· **`map/`**（建图输出）

## 导航调参文件（base + overlay）

`config/nav/base.yaml` 是公共层（机器人外形、控制频率、规划器图、MCL 默认值等）。每个 profile 只写自己需要覆盖的键，launch 用 `parameters=[base.yaml, profile.yaml]` 顺序加载，后者覆盖前者：

| Profile | 说明 |
|------|------|
| `mid360_mapping` | 边建图边导航 |
| `mid360_mapping_with_camera` | mapping_nav + RealSense + 语义点云 |
| `mid360_localization` | 纯 LiDAR 定位导航（默认主线） |
| `mid360_localization_with_camera` | 定位 + 语义点云 |
| `mid360_localization_with_depth_camera` | 定位 + 深度相机层（不带语义） |

切换调参用 `nav_profile`：

```bash
ros2 launch dddnav_bringup localization.launch.py nav_profile:=mid360_localization_with_depth_camera
ros2 launch dddnav_bringup mapping_nav.launch.py  nav_profile:=mid360_mapping_with_camera
```

要用包外的实验性 yaml 直接传绝对路径：

```bash
ros2 launch dddnav_bringup localization.launch.py nav_profile:=/abs/path/custom.yaml
```

复制一份 `config/nav/mid360_localization.yaml` 到同目录改个名（如 `myrobot_indoor.yaml`），写差异部分即可作为新 profile。`p2p_move_base/config/` 现在只留 Go2 专用 yaml。

## 参数示例

```bash
ros2 launch dddnav_bringup mapping.launch.py fastlio_config:=/path/to/velodyne.yaml
ros2 launch dddnav_bringup mapping.launch.py auto_save_on_exit:=true
ros2 launch dddnav_bringup localization.launch.py nav_profile:=mid360_localization_with_camera
```
