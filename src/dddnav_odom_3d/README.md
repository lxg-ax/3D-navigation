# dddnav_odom_3d

3D 里程计示例包：差速 / 滑移车的 2D 轮速 + IMU 姿态积成简单 3D pose。**教学向**，不能当生产里程计。

## 原理

* 轮速给出 2D 平面速度 `(vx, ω)`
* IMU 给出当前 roll / pitch / yaw（需要事先做姿态滤波，否则会漂）
* 在 base 系按 IMU 姿态把 2D 速度旋到 world 系积分得到 3D 位置

## 限制

* 假设无侧向滑移、无车体抬起
* 没做轮速 / IMU 时间同步（demo 用 bag 时刚好对齐）
* 不能替代 FAST-LIO / 任何紧耦合 LIO

主线 SLAM 请用 `FAST_LIO` + `LIO-SAM`，见 [根 README](../../README.md)。

## Demo

`example_odom_3d_launch.py` 会 `ros2 bag play bag_files/rosbag2_odom2d_imu/`。bag 不入库，请自己 record 或改 launch 路径。

```bash
ros2 launch dddnav_odom_3d example_odom_3d_launch.py
```
