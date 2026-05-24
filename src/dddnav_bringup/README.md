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
# ctrl-C 退出时自动调上面两个 service，覆盖到 dddnav_bringup/map/
```
`auto_save_on_exit:=true` 时会在启动前清空 `map/` 下的旧 pcd（`pcd/`、`poses.pcd`、`edges.pcd`、`map.pcd`、`ground.pcd` 以及 LIO-SAM 的 `GlobalMap/CornerMap/SurfMap/trajectory/transformations.pcd`），不删整个 `map/` 目录（colcon symlink-install 的链接还在用）。LIO-SAM 的输出会落到 `map/lio_sam/` 子目录，避免和位姿图互删。`mapping_nav.launch.py` 同样支持这个参数。

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

| 模式 | Launch | 默认 nav profile |
|------|--------|---------------------------------------------|
| 建图+视觉 | `mapping_with_camera.launch.py` | — |
| 建图导航+视觉 | `mapping_nav_with_camera.launch.py` | `mid360_mapping_with_camera` |
| 定位+视觉 | `localization_with_camera.launch.py` | `mid360_localization_with_camera` |

## 目录

`launch/` · `config/`（含 `config/nav/` 导航调参）· `rviz/`（mapping / mapping_nav / localization）· **`map/`**（建图输出）

## 导航调参文件

所有 Mid360 主线的导航调参 yaml 都在 `config/nav/`：

| Profile | 说明 |
|------|------|
| `mid360_mapping` | 边建图边导航（mapping_nav 模式） |
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

复制一份 `config/nav/mid360_localization.yaml` 到同目录改个名（如 `myrobot_indoor.yaml`），然后 `nav_profile:=myrobot_indoor` 即可。`p2p_move_base/config/` 现在只留 Go2 专用 yaml。

## 参数示例

```bash
ros2 launch dddnav_bringup mapping.launch.py fastlio_config:=/path/to/velodyne.yaml
ros2 launch dddnav_bringup mapping.launch.py auto_save_on_exit:=true
ros2 launch dddnav_bringup localization.launch.py nav_profile:=mid360_localization_with_camera
```
