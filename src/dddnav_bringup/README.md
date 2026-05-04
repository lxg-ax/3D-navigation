# dddnav_bringup

一键启动：建图、边建图边导航、定位导航；每种有纯 LiDAR 与带 RealSense+DDRNet 版本。总览与路径约定：[根 README](../../README.md)。

**数据：** 位姿图等写到 **`map/`**（安装后 `share/dddnav_bringup/map`）。`localization*.launch.py` 为 `mcl_3dl` 注入 `sub_maps.pose_graph_dir`；`mapping*.launch.py` 为 LIO-SAM 注入 `savePCDDirectory`，均指向该目录。仓库**不提交** `map/` 下的 **`*.pcd`**（体积大、与环境相关）；克隆后需先**自行建图并保存**，再跑定位。

## 命令

**建图**
```bash
ros2 launch dddnav_bringup mapping.launch.py
ros2 launch dddnav_bringup mapping_with_camera.launch.py
```
保存：
```bash
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2}"
ros2 service call /save_liosam_posegraph std_srvs/srv/Empty {}
```

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

## 带相机 (`*_with_camera`)

需要 RealSense（D435/D455）、DDRNet 的 TensorRT 引擎（在目标 GPU 上 `trtexec` 转 ONNX，见根 README「Semantic」）。可选改 TF：`launch/common_camera_nodes.py` 里 `camera_tf_nodes(...)`。

数据流：RGB/深度 → `ddrnet_ros_img_sub.py` → mask → `semantic_segmentation2point_cloud` → 话题 **`/sematic_segmentation_point_cloud`**（历史拼写）→ `perception_3d::DepthCameraLayer`。

| 模式 | Launch | YAML（在 `dddnav_p2p_move_base/config/`） |
|------|--------|---------------------------------------------|
| 建图+视觉 | `mapping_with_camera.launch.py` | — |
| 建图导航+视觉 | `mapping_nav_with_camera.launch.py` | `mid360_mapping_with_camera.yaml` |
| 定位+视觉 | `localization_with_camera.launch.py` | `mid360_localization_with_camera.yaml` |

## 目录

`launch/` · `config/` · `rviz/`（mapping / mapping_nav / localization）· **`map/`**（建图输出）

## 参数示例

```bash
ros2 launch dddnav_bringup mapping.launch.py fastlio_config:=/path/to/velodyne.yaml
ros2 launch dddnav_bringup localization.launch.py nav_config:=/path/to/nav.yaml
```
