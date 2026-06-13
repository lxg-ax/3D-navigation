# dddnav_std_descriptor

[hku-mars/STD](https://github.com/hku-mars/STD)（Stable Triangle Descriptor）的 ROS 2 移植版。**纯支撑库，没有节点**，被建图与定位两条主线共同调用。

## 功能

* 把雷达点云抽成 6-DoF 旋转/平移不变的三角描述子（边长 + 角度 + 三顶点）
* 三边长 hash 检索 → plane-ICP 验证 → 6-DoF 相对位姿候选
* 自带磁盘持久化（`std_db_io.h`），LIO-SAM 在 `saveMap` 时整个 DB 落盘 `std_db.bin`，定位起来时整 DB load
* 把上游 ROS 1 NodeHandle / Publisher 耦合剥掉，并兼容 Ceres 2.0 / 2.1+

## 在系统中的角色

```
建图：LIO-SAM 关键帧 ─► STDescManager.GenerateSTDescs ─► std_db.bin（落到 map/lio_sam/）

定位：dddnav_utils/std_global_init ─► STDescManager.SearchLoop ─► /initial_3d_pose
       ↑                              ↑
       启动期一次性发种子              运行期 watchdog 持续巡检
```

替代了原本的 Scan Context（依赖列移位恢复 yaw）。STD 直接给 6-DoF 相对位姿，长走廊 / 对称楼道下误匹配率显著更低，候选还能用 plane-ICP score 做硬阈值过滤。

参与流程 / 调用点见 [`dddnav_utils` README](../dddnav_utils/README.md#std-全局初始化--在线重定位巡检)。

## 公共 API

| Header | 用途 |
|--------|------|
| `STDesc.h` | `STDescManager`、`ConfigSetting`、`STDesc`，以及 `down_sampling_voxel` / `getPlane` 等几何工具 |
| `std_db_io.h` | `saveStdDatabase` / `loadStdDatabase`，bit-exact 往返，文件头 `magic='DSTD' / version=1` |
| `std_config_loader.h` | `loadStdConfig(node, cfg)`，把 `std.*` 参数从 rclcpp Node 一次性灌进 `ConfigSetting` |

DB 文件大小经验：~50 STD/frame × ~1k 关键帧 ≈ 几 MB，整 DB 直接进内存。

## 关键参数

`std.*` 命名空间，由调用方（`lio_sam_mapOptimization` / `std_global_init`）声明，默认值取上游 baseline。常调：

| Key | 默认 | 含义 |
|-----|------|------|
| `std.voxel_size` | 2.0 | 平面检测 voxel 边长（m） |
| `std.plane_detection_thre` | 0.01 | 平面拟合特征值阈值，越小越严 |
| `std.corner_thre` | 10.0 | 角点提取阈值 |
| `std.maximum_corner_num` | 100 | 单帧最多保留多少角点（控规模） |
| `std.proj_image_resolution` | 0.5 | 投影图分辨率（m/cell），影响角点定位精度 |
| `std.ds_size` | 0.5 | 输入降采样 voxel（m） |
| `std.stop_skip_enable` | 0 | 静止时跳过描述子提取（建图阶段开） |

完整列表见 `include/dddnav_std_descriptor/std_config_loader.h`。

## 依赖

`rclcpp`、Eigen、PCL、Ceres ≥ 2.0。

## 来源

上游：[hku-mars/STD](https://github.com/hku-mars/STD)（BSD-3-Clause）。本包改动均为 ROS 2 接口适配与持久化层，算法本体未动。
