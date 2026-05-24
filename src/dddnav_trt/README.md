# dddnav_trt

可选的 YOLOv8 + TensorRT C++ 库。**默认主线不用**，需要时单独打开。

## 原理

* 上游 `tensorrt-cpp-api`（vendored 在 `src/yolo/yolo_trt/tensorrt-cpp-api/`）做 TRT engine 加载与 IO
* YOLOv8 模型用于 2D 检测（人 / 车 / 障碍物）
* 推理结果通常配合深度图升 3D，给动态层 / 跟踪用

## 在系统中的角色

可选模块。现有定位 / 导航主线（FAST-LIO + LIO-SAM + MCL + perception_3d）不依赖它；如果业务需要"识别动态物体并差异化避障"，再开。

## 编译

```bash
cd /path/to/REPO
source /opt/ros/humble/setup.bash
colcon build --packages-select dddnav_trt --cmake-args -DTRT_ENABLED=ON -DCMAKE_BUILD_TYPE=Release
```

## 注意

* TensorRT 与 CUDA 版本必须与本机 driver 匹配（`dddnav_docker/Dockerfile_x64_cuda` 已配好一个组合）
* engine 不能跨 GPU 拷贝，部署到目标机器再转

详细 vendored API 说明：`src/yolo/yolo_trt/tensorrt-cpp-api/README.md`。
