# dddnav_bringup

整套导航栈的入口包。提供建图 / 边建边导航 / 定位导航三类 launch（每类含纯 LiDAR 与带 RealSense+DDRNet 两份），把 SLAM 前后端、定位、ESKF 融合、感知、规划、监控按统一拓扑拼起来。

总览：[根 README](../../README.md)。调参索引：[`config/PARAMETERS.md`](config/PARAMETERS.md)。

## 数据约定

* 位姿图、地图、STD 数据库统一写到 `map/`（安装后 `share/dddnav_bringup/map`）
* `localization*.launch.py` 给 `mcl_3dl` 注入 `sub_maps.pose_graph_dir` 指向该目录
* `mapping*.launch.py` 给 LIO-SAM 注入 `savePCDDirectory` 指向该目录
* 仓库**不提交** `map/` 下的 `*.pcd`；克隆后先建图再跑定位

## 启动命令

### 建图

```bash
ros2 launch dddnav_bringup mapping.launch.py                 # 纯 LiDAR
ros2 launch dddnav_bringup mapping_with_camera.launch.py     # + RealSense + DDRNet
```

保存（手动，**顺序敏感**——LIO-SAM 的 save service 会清空自己的目录，颠倒会把已写好的 `pcd/N_*.pcd` 一锅端）：

```bash
ros2 service call /save_liosam_posegraph std_srvs/srv/Empty {}
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2}"
```

保存（自动）：

```bash
ros2 launch dddnav_bringup mapping.launch.py auto_save_on_exit:=true
# Ctrl-C 时由 save_map_on_exit.py 按顺序两个 service 落盘；启动前会清旧图。
```

### 边建图边导航

```bash
ros2 launch dddnav_bringup mapping_nav.launch.py
ros2 launch dddnav_bringup mapping_nav_with_camera.launch.py
```

回环修正 `map→odom` 后，路径由 `p2p_global_plan_manager` 5Hz 重查刷新。

### 定位导航（需先有 `map/` 数据）

```bash
ros2 launch dddnav_bringup localization.launch.py
ros2 launch dddnav_bringup localization_with_camera.launch.py
```

启动时 `std_global_init` 用 STD 三角描述子自动找初始位姿，**无需手动点 RViz**。无 `std_db.bin` 时静默退出，`runtime.yaml.initial_pose` 兜底。

定位流程同时启两个 watchdog：

* `slam_health_monitor.py`：odom topic / TF 边 liveness、`map→odom` 跳变、`/odom_filtered` cov trace
* `nav_perf_monitor.py`：`cmd_vel` / 路径速率、FAST-LIO 残差 / 有效特征数

两者推到 `/diagnostics`。

## 带相机分支（`*_with_camera`）

依赖：

* RealSense（D435/D455），默认 848×480
* DDRNet 的 TensorRT 引擎，需在**目标 GPU** 上 `trtexec` 转 ONNX（见 `dddnav_semantic_segmentation/README.md`）
* 相机外参在 [`config/reality/runtime.yaml`](config/reality/runtime.yaml) `camera_mount`

数据流：

```
RGB / depth ─► ddrnet_ros_img_sub.py ─► mask
                                       │
depth ─────────────────────────────────┤
                                       ▼
        semantic_segmentation2point_cloud
                                       │
                                       ▼
                /sematic_segmentation_point_cloud
                                       │
                                       ▼
                perception_3d::DepthCameraLayer
```

| 模式 | Launch | 默认 nav profile |
|------|--------|-----|
| 建图+视觉 | `mapping_with_camera.launch.py` | — |
| 建图导航+视觉 | `mapping_nav_with_camera.launch.py` | `mid360_mapping_with_camera` |
| 定位+视觉 | `localization_with_camera.launch.py` | `mid360_localization_with_camera` |

> 相机分支已编译、launch 拓扑通；真机端到端联调还没在主线完成，落地前请按场景验证。

## Launch 公共组件

`launch/common_nodes.py` 抽出每个 launch 共用的部件，要改某个节点的拓扑改一处即可。

| Helper | 内容 |
|--------|------|
| `lidar_driver_and_tf(rt)` | Livox driver + `base_link→livox_frame` static TF |
| `lidar_front_end(rt, fastlio_yaml)` | livox→liosam 桥接 + FAST-LIO |
| `liosam_back_end(...)` | LIO-SAM 4 节点 + `liosam_to_posegraph` + `slam_health_monitor`（建图模式 `ok_timeout=1.0s/fail_timeout=3.0s`） |
| `localization_stack(...)` | MCL 3DL + pose_fusion + mcl_feature + `std_global_init` + 兜底初始位姿 |
| `nav_stack(rt, params)` | global_planner + p2p_move_base + clicked2goal + `nav_perf_monitor` |
| `auto_save_actions(...)` | 启动前清旧图 + Ctrl-C 触发 `save_map_on_exit.py` |
| `rviz_action(rt, rviz_yaml)` | RViz 延时启动 |

## 目录

```
launch/   每个模式一个入口 + common_nodes.py / common_camera_nodes.py / bringup_paths.py
config/
  PARAMETERS.md             调参索引（参数 → 来源文件）
  nav_base.yaml             导航公共参数
  reality/
    runtime.yaml            安装外参 / 启动延时 / 初始位姿 / 驱动频率
    keyframes_mid360.yaml   关键帧抽取阈值
    pose_fusion.yaml        ESKF Q/R / ZUPT / 自适应 Q / 退化状态机
    slam/                   FAST-LIO + LIO-SAM 调参（IMU_NOTES.md 含跨算法对照）
    nav/<profile>.yaml      各模式 overlay
    tuning/                 现场调参 overlay 模板
rviz/     mapping / mapping_nav / localization 三套
map/      建图输出（`*.pcd` 不入库）
```

## 排错

### 建图启动后 `slam_health_monitor` 一直刷 `no message` / `tf map->odom not available`

`livox_pc2_to_liosam` 两个 publisher 与 LIO-SAM `imageProjection` 的 QoS 必须都是 RELIABLE，错配时 DDS 拒连，整条管道空转但节点都活着。诊断：

```bash
ros2 topic hz /livox/lidar_liosam            # 应 ~10 Hz
ros2 topic info -v /livox/lidar_liosam       # publisher/subscriber 都应 RELIABLE
ros2 topic hz /lio_sam/deskew/cloud_deskewed
ros2 topic hz /lio_sam/mapping/odometry
```

修法：保持 `dddnav_utils/src/livox_pc2_to_liosam.cpp` 的 `pub_qos.reliable()` 不要回退。

### 雷达静止时偶发 `[tf map->odom] stamp_age=0.5s`

LIO-SAM `mappingProcessInterval=0.1` + 单次优化偶发 100~150 ms 会触达旧默认 `ok_timeout=0.5s`。建图链已放宽到 `ok_timeout=1.0s / fail_timeout=3.0s`，见 `common_nodes.py::liosam_back_end`。

## 导航 profile

`config/nav_base.yaml` 是公共层，每个 profile 只写差异，launch 顺序加载后者覆盖前者。

| Profile | 说明 |
|------|------|
| `mid360_mapping` | 边建图边导航 |
| `mid360_mapping_with_camera` | 上 + RealSense + 语义点云 |
| `mid360_localization` | 纯 LiDAR 定位（默认主线） |
| `mid360_localization_with_camera` | 定位 + 语义点云 |
| `mid360_localization_with_depth_camera` | 定位 + 深度相机层（无语义） |

切换：

```bash
ros2 launch dddnav_bringup localization.launch.py nav_profile:=mid360_localization_with_depth_camera
ros2 launch dddnav_bringup localization.launch.py nav_profile:=/abs/path/custom.yaml
```

新做 profile：复制 `config/reality/nav/mid360_localization.yaml` 改名后写差异。
