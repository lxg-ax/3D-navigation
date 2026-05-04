# DDDNAV ODOM 3D

ROS 包 **`dddnav_odom_3d`**：差速/滑移转向车把 2D 轮速 + IMU 欧拉角积成简单 3D 里程计示例。教学向，**不能当量产融合**。总览：[根 README](../../README.md)。

限制：无侧向控制假设、无滑移抬起模型；IMU 姿态需滤波，否则漂。

<p align="center"><img src="https://github.com/dfl-rlab/dddnav_documentation_materials/blob/main/odom_3d/Differential_drive_robot_3D_odometry_approximation_.png" width="520" height="620"/></p>

## 运行

`REPO` = 仓库根。`example_odom_3d_launch.py` 会 `ros2 bag play` **`bag_files/rosbag2_odom2d_imu/`**；该目录下的 **`.db3` / `metadata.yaml` 不入库**，请在本机 `ros2 bag record` 生成同名结构，或改 launch 里路径。

```bash
cd /path/to/REPO/dddnav_docker/docker_file && ./build.bash
cd /path/to/REPO/dddnav_docker && ./run_demo.bash
cd /path/to/REPO && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch dddnav_odom_3d example_odom_3d_launch.py
```

<p align="center"><img src="https://github.com/dfl-rlab/dddnav_documentation_materials/blob/main/odom_3d/3d_odom_demo.gif" width="700" height="420"/></p>
