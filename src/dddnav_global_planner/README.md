# dddnav_global_planner

ROS 包 **`global_planner`**。地面点云上的图搜索全局路径；接 **`perception_3d`** 动静态图。总览：[根 README](../../README.md)。

<p align="center"><img src="https://github.com/dfl-rlab/dddnav_documentation_materials/blob/main/global_planner/global_plan.png" width="640"/></p>
<p align="center"><img src="https://github.com/dfl-rlab/dddnav_documentation_materials/blob/main/global_planner/boundary_annotated.png" width="640"/></p>

<p align="center"><img src="https://github.com/dfl-rlab/dddnav_documentation_materials/blob/main/global_planner/global_planner_diagram.png" width="640" height="420"/></p>

## Demo

`REPO` = 含 `dddnav_docker/` 的根目录。

```bash
cd /path/to/REPO/dddnav_docker/docker_file && ./build.bash
cd /path/to/REPO/dddnav_docker && ./run_demo.bash
cd /path/to/REPO && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch global_planner path_planning_on_static_layer.launch
```

RViz 里用 **Publish Point** 发目标。

<p align="center"><img src="https://github.com/dfl-rlab/dddnav_documentation_materials/blob/main/global_planner/global_planner_demo.png" width="640" height="400"/></p>
