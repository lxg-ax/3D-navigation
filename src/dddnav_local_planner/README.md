# dddnav_local_planner

Metapackage：含 **`local_planner`**、**`mpc_critics`**、**`trajectory_generators`**、**`recovery_behaviors`**、**`base_trajectory`**。3D 碰撞/评分（cuboid vs 点云），轨迹族可配不同 critic。差速 `dd_simple_trajectory_generator` 考虑电机极限耦合。总览：[根 README](../../README.md)。

<p align="center"><img src="https://github.com/dfl-rlab/dddnav_documentation_materials/blob/main/local_planner/local_planner_play_ground_annotated.png" width="720" height="420"/></p>

## Playground demo

`REPO` = 仓库根。

```bash
cd /path/to/REPO/dddnav_docker/docker_file && ./build.bash
cd /path/to/REPO/dddnav_docker && ./run_demo.bash
cd /path/to/REPO && source /opt/ros/humble/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch local_planner local_planner_play_ground.launch
```

RViz **Publish Point** 发局部目标。

<p align="center"><img src="https://github.com/dfl-rlab/dddnav_documentation_materials/blob/main/local_planner/local_planner_play_ground.gif" width="700" height="440"/></p>
