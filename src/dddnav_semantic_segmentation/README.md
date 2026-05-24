# dddnav_semantic_segmentation

DDRNet + TensorRT 的语义分割管线：RGB → mask → 与深度对齐 → 语义点云。

## 原理

```
RealSense RGB ─► ddrnet_ros_img_sub.py ─► mask（每像素 class id）
RealSense depth ───────────────────────┐
                                       ▼
                semantic_segmentation2point_cloud
                                       │
                                       ▼
                       /sematic_segmentation_point_cloud
                       （历史拼写，PointXYZRGB + class）
```

* **DDRNet**：双分辨率分割网络，TensorRT 推理速度约束在 30Hz
* **对齐**：用相机内参把 mask 和 depth 像素 1:1 对应，按 `voxel_size` / `sample_step` 降采样
* **类别过滤**：`exclude_class` 把背景之类丢掉

## 在系统中的角色

`*_with_camera` 模式下作为感知输入：

```
DDRNet 语义点云 ─► perception_3d::DepthCameraLayer ─► local_planner / global_planner
```

主要价值是补 LiDAR 在近距离 / 透明物 / 玻璃面的盲区，以及给后续动态物体 / traversability 留语义入口。

## 依赖

* RealSense（默认 848×480，见 `launch/rs_semantic_segmentaton_trt_launch.py`）
* 在**目标 GPU** 上自己用 `trtexec` 把 ONNX 转 `.trt`（不能跨 GPU 拷）

## 转 TRT 引擎

```bash
cd src/dddnav_semantic_segmentation/model   # 容器内：/root/dddnav_navigation/src/...
/usr/src/tensorrt/bin/trtexec \
  --onnx=ddrnet_23_slim_dualresnet_citys_best_model_424x848.onnx \
  --saveEngine=ddrnet_23_slim_dualresnet_citys_best_model_424x848.trt
```

## 启动

```bash
colcon build --packages-select dddnav_semantic_segmentation --symlink-install
source install/setup.bash
ros2 launch dddnav_semantic_segmentation rs_semantic_segmentaton_trt_launch.py
```

Bag 示例：`bag_exclude_ss_trt_launch.py` / `bag_semantic_segmentaton_trt_launch.py`，默认读 `~/dddnav_bags/...`。

类别颜色映射：`data/colors_mapillary.csv`。

## 当前状态

代码 / 编译路径已经接通；真机带 RealSense 的端到端联调还没在 dddnav 主线上完成（见根 README 的「相机分支」说明）。LiDAR 主线工作正常。
