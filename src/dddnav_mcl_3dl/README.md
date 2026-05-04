# mcl_3dl

ROS 包 **`mcl_3dl`**（目录 `dddnav_mcl_3dl/`）。基于 [at-wat/mcl_3dl](https://github.com/at-wat/mcl_3dl) 改地面车版本。总览：[根 README](../../README.md)。

**位姿图：** 默认 Mid360 线用 **LIO-SAM + `liosam_to_posegraph`** 写到 **`dddnav_bringup/map`**，`localization.launch.py` 设 `sub_maps.pose_graph_dir`。老 **Go2 / bag** 演示仍可能走 **`lego_loam_bor` + `mcl_feature`**。子图按 `sub_maps.pose_graph_dir` 加载，不是单张大 PCD。

<p align="center"><img src="https://github.com/dfl-rlab/dddnav_documentation_materials/blob/main/dddnav_mcl_3dl/dddnav_mcl_3dl.gif" width="640" height="400"/></p>
<p align="center"><img src="https://github.com/dfl-rlab/dddnav_documentation_materials/blob/main/dddnav_mcl_3dl/mcl_3dl_diagram.png" width="640" height="400"/></p>

相对原版要点：ROS 2；粒子更新按行驶距离/转角减负；子图降算力；地面约束；聚类+法向的打分减轻远点稀疏与“假不动”。

子图尺度量级：`2 * lidar_detection_distance + 2 * sub_map_search_radius`。

<p align="center"><img src="https://github.com/dfl-rlab/dddnav_documentation_materials/blob/main/dddnav_mcl_3dl/mcl_3dl_submap_illustration.png" width="850" height="260"/></p>

## Bag 演示

`REPO` = 仓库根。

```bash
cd /path/to/REPO/dddnav_docker/docker_file && ./build.bash
cd /path/to/REPO/dddnav_docker && ./run_demo.bash
cd /path/to/REPO/src/dddnav_mcl_3dl && ./download_files.bash
cd /path/to/REPO && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch mcl_3dl mcl_3dlXfeatureXbag.launch
```

另开终端：`docker exec -it dddnav_humble_dev bash`（容器名按实际），内执行：

```bash
cd /root/dddnav_navigation && source install/setup.bash
cd /root/dddnav_bags && ros2 bag play benanli_detention_basin_localization
```

主机 bag 放 **`~/dddnav_bags`**。

<p align="center"><img src="https://github.com/dfl-rlab/dddnav_documentation_materials/blob/main/dddnav_mcl_3dl/mcl_initial_pose.png" width="640" height="400"/></p>
