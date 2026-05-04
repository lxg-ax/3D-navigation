# dddnav_trt

可选 **YOLOv8 + TensorRT** C++ 库。默认建图仍是 FAST-LIO2 + LIO-SAM；见 [根 README](../../README.md)。

```bash
cd /path/to/REPO
source /opt/ros/humble/setup.bash
colcon build --packages-select dddnav_trt --cmake-args -DTRT_ENABLED=ON -DCMAKE_BUILD_TYPE=Release
```

TensorRT/CUDA 需与本机驱动一致。`src/yolo/yolo_trt/tensorrt-cpp-api/README.md` 为 vendored API 说明。
