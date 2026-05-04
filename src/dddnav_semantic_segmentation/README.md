# dddnav_semantic_segmentation

DDRNet + TensorRT：对齐深度与分割 mask，发彩色点云。总览：[根 README](../../README.md)。

<p align="center"><img src="https://github.com/dfl-rlab/dddnav_documentation_materials/blob/main/dddnav_semantic_segmentation/dddnav_semantic_segmentation_to_pointcloud.gif" width="640" height="440"/></p>

## 依赖

- RGB-D（默认 launch 用 848×480，见 `launch/rs_semantic_segmentaton_trt_launch.py`）
- 在**目标 GPU** 上从 `model/*.onnx` 生成 `.trt`（不可跨 GPU 乱拷）

## TRT 引擎

```bash
cd src/dddnav_semantic_segmentation/model   # 容器内：/root/dddnav_navigation/src/...
/usr/src/tensorrt/bin/trtexec \
  --onnx=ddrnet_23_slim_dualresnet_citys_best_model_424x848.onnx \
  --saveEngine=ddrnet_23_slim_dualresnet_citys_best_model_424x848.trt
```

## 运行

```bash
colcon build --packages-select dddnav_semantic_segmentation --symlink-install
source install/setup.bash
ros2 launch dddnav_semantic_segmentation rs_semantic_segmentaton_trt_launch.py
```

Bag 示例：`bag_exclude_ss_trt_launch.py` / `bag_semantic_segmentaton_trt_launch.py`，默认读 **`~/dddnav_bags/...`**（Docker 见 `dddnav_docker` 挂载说明）。

类别颜色：`data/colors_mapillary.csv`。

## 图

<p align="center"><img src="https://github.com/dfl-rlab/dddnav_documentation_materials/blob/main/dddnav_semantic_segmentation/dddnav%20semantic%20segmentation.png" width="780" height="560"/></p>

GIF/大图托管在 [dddnav_documentation_materials](https://github.com/dfl-rlab/dddnav_documentation_materials)。
