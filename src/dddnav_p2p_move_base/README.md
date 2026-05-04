# DDDNAV P2P Move Base

ROS 包 **`p2p_move_base`**。3D 点到点：先转向再走、阻塞等待后重规划、可中断的 recovery（action）。Recovery 插件见 [dddnav_local_planner](../dddnav_local_planner/)。总览：[根 README](../../README.md)。

与 classic `move_base` 差异简述：旋转 shim 类似 [nav2_rotation_shim_controller](https://github.com/ros-navigation/navigation2/tree/main/nav2_rotation_shim_controller)；`waiting_patience` 后重算全局；取消 goal 会先停 recovery。TODO：recovery 全可配 YAML（当前主要原地转）。

## Go2 + bag 老演示

`REPO` = 仓库根。

```bash
cd /path/to/REPO/dddnav_docker/docker_file && ./build.bash
cd /path/to/REPO/dddnav_docker && ./run_demo.bash
cd /path/to/REPO/src/dddnav_mcl_3dl && ./download_files.bash
cd /path/to/REPO && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch p2p_move_base go2_localization.launch
```

`go2_localization.launch` → `go2_localization.launch.py` 设置 `sub_maps.pose_graph_dir` → **`dddnav_bringup/map`**。Mid360 主线用 **`ros2 launch dddnav_bringup localization.launch.py`**。

**播 bag：** 另开 shell `docker exec -it dddnav_humble_dev bash`（容器名按实际改），然后：

```bash
cd /root/dddnav_navigation && source install/setup.bash
cd /root/dddnav_bags && ros2 bag play benanli_detention_basin_localization
```

主机把 bag 放 **`~/dddnav_bags`** 以便默认挂载到容器 `/root/dddnav_bags`。

<p align="center"><img src="https://github.com/dfl-rlab/dddnav_documentation_materials/blob/main/p2p_move_base/p2p_move_base_annotated.png" width="720" height="420"/></p>

[![video](https://github.com/dfl-rlab/dddnav_documentation_materials/blob/main/p2p_move_base/p2p_move_base_video.png)](https://www.youtube.com/watch?v=7zyrRIE7eaU)
