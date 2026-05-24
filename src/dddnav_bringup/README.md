# dddnav_bringup

整套导航栈的入口包。提供建图 / 边建边导航 / 定位导航三类 launch（每类都有纯 LiDAR 与带 RealSense+DDRNet 两份），把 SLAM 前后端、定位、ESKF 融合、感知、规划、监控按统一拓扑拼起来。

总览：[根 README](../../README.md)。

## 数据约定

* 位姿图、地图、Scan Context 数据库统一写到 **`map/`**（安装后 `share/dddnav_bringup/map`）
* `localization*.launch.py` 给 `mcl_3dl` 注入 `sub_maps.pose_graph_dir` 指向该目录
* `mapping*.launch.py` 给 LIO-SAM 注入 `savePCDDirectory` 指向该目录
* 仓库**不提交** `map/` 下的 `*.pcd`（体积大、与环境相关）；克隆后先建图再跑定位

## 启动命令

### 建图

```bash
ros2 launch dddnav_bringup mapping.launch.py                 # 纯 LiDAR
ros2 launch dddnav_bringup mapping_with_camera.launch.py     # + RealSense + DDRNet
```

保存（手动）：

```bash
ros2 service call /save_liosam_posegraph std_srvs/srv/Empty {}
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2}"
```

> 顺序：先 `/save_liosam_posegraph` 再 `/lio_sam/save_map`。LIO-SAM 的 save service 会 `rm -r` 自己的 `savePCDDirectory`，颠倒会把 `liosam_to_posegraph` 已写好的 `pcd/N_*.pcd` 一锅端掉。

保存（自动）：

```bash
ros2 launch dddnav_bringup mapping.launch.py auto_save_on_exit:=true
# Ctrl-C 时由 dddnav_utils/save_map_on_exit.py 按顺序两个 service 落盘
```

`auto_save_on_exit:=true` 启动前会清空旧 pcd，退出时由 rclpy 节点（不再是 bash 链）按顺序保存，launch tear-down 影响不到。LIO-SAM 输出落到 `map/lio_sam/` 子目录，避免和位姿图互删。`mapping_nav.launch.py` 同样支持。

### 边建图边导航

```bash
ros2 launch dddnav_bringup mapping_nav.launch.py
ros2 launch dddnav_bringup mapping_nav_with_camera.launch.py
```

回环修正 `map→odom` 后路径会被 `p2p_global_plan_manager` 5Hz 重查自动刷新。

### 定位导航（需先有 `map/` 数据）

```bash
ros2 launch dddnav_bringup localization.launch.py
ros2 launch dddnav_bringup localization_with_camera.launch.py
```

启动时 `sc_global_init` 用 Scan Context 自动找初始位姿，**无需手动点 RViz**。无 `sc_db.bin` 时节点静默退出，`runtime.yaml.initial_pose` 仍兜底。

定位流程同时启两个 watchdog：

* `slam_health_monitor.py`：odom 话题 / TF 边 liveness、`map→odom` 跳变、`/odom_filtered` cov trace
* `nav_perf_monitor.py`：`cmd_vel` / 全局规划路径速率，FAST-LIO 残差 / 有效特征数

两者都把状态推到 `/diagnostics`，foxglove / RViz Diagnostic 面板直接订阅。

## 带相机分支（`*_with_camera`）

依赖：

* RealSense（D435/D455），默认 848×480
* DDRNet 的 TensorRT 引擎，需在**目标 GPU** 上 `trtexec` 转 ONNX（见根 README「Semantic」段或 `dddnav_semantic_segmentation/README.md`）
* 相机外参在 [`config/runtime.yaml`](config/runtime.yaml) 的 `camera_mount`（与 `lidar_mount` 同级），改 yaml 即可，不动 launch

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
|------|--------|---------------------------------------------|
| 建图+视觉 | `mapping_with_camera.launch.py` | — |
| 建图导航+视觉 | `mapping_nav_with_camera.launch.py` | `mid360_mapping_with_camera` |
| 定位+视觉 | `localization_with_camera.launch.py` | `mid360_localization_with_camera` |

> 当前状态：相机分支已编译、launch 拓扑通；真机端到端联调还没在主线完成，落地前请按场景验证。

## Launch 公共组件

`launch/common_nodes.py` 把每个入口 launch 共用的部件抽出来，每个入口 launch 现在只是这些 helper 的拼装。要改某个节点的拓扑，到 `common_nodes.py` 改一处即可。

| Helper | 内容 |
|--------|------|
| `lidar_driver_and_tf(rt)` | Livox driver + `base_link→livox_frame` static TF |
| `lidar_front_end(rt, fastlio_yaml)` | livox→liosam 桥接 + FAST-LIO |
| `liosam_back_end(...)` | LIO-SAM 4 节点 + `liosam_to_posegraph` + `slam_health_monitor` |
| `localization_stack(...)` | MCL 3DL + pose_fusion + mcl_feature + `sc_global_init` + 兜底初始位姿 |
| `nav_stack(rt, params)` | global_planner + p2p_move_base + clicked2goal + `nav_perf_monitor` |
| `auto_save_actions(...)` | 启动前清旧图 + Ctrl-C 触发 `save_map_on_exit.py` |
| `rviz_action(rt, rviz_yaml)` | RViz 延时启动 |

## 目录

```
launch/   每个模式一个入口 + common_nodes.py / common_camera_nodes.py / bringup_paths.py
config/
  runtime.yaml              安装外参 / 启动延时 / 初始位姿 / 驱动频率
  keyframes_mid360.yaml     关键帧抽取阈值
  nav/
    base.yaml               导航公共参数（机器人外形、控制频率、规划器、MCL 默认值）
    mid360_*.yaml           各模式 overlay
rviz/     mapping / mapping_nav / localization 三套
map/      建图输出（`*.pcd` 不入库）
scripts/  辅助脚本
```

## 导航调参（base + overlay）

`config/nav/base.yaml` 是公共层。每个 profile 只写差异，launch 用 `parameters=[base.yaml, profile.yaml]` 顺序加载，后者覆盖前者：

| Profile | 说明 |
|------|------|
| `mid360_mapping` | 边建图边导航（mapping_nav 模式） |
| `mid360_mapping_with_camera` | 上行 + RealSense + 语义点云 |
| `mid360_localization` | 纯 LiDAR 定位导航（默认主线） |
| `mid360_localization_with_camera` | 定位 + 语义点云 |
| `mid360_localization_with_depth_camera` | 定位 + 深度相机层（不带语义） |

切换：

```bash
ros2 launch dddnav_bringup localization.launch.py nav_profile:=mid360_localization_with_depth_camera
ros2 launch dddnav_bringup mapping_nav.launch.py  nav_profile:=mid360_mapping_with_camera
```

或直接传绝对路径：

```bash
ros2 launch dddnav_bringup localization.launch.py nav_profile:=/abs/path/custom.yaml
```

新做一个 profile：复制 `config/nav/mid360_localization.yaml` 改名（如 `myrobot_indoor.yaml`），写差异即可。`p2p_move_base/config/` 现在只留 Go2 专用 yaml。

## 参数示例

```bash
ros2 launch dddnav_bringup mapping.launch.py fastlio_config:=/path/to/velodyne.yaml
ros2 launch dddnav_bringup mapping.launch.py auto_save_on_exit:=true
ros2 launch dddnav_bringup localization.launch.py nav_profile:=mid360_localization_with_camera
```
