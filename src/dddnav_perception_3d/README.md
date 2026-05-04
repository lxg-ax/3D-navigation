# dddnav_perception_3d

ROS 包名 **`perception_3d`**。3D 点云上的标记/清除、限速区、禁入区等。总览与 bag 路径：[根 README](../../README.md)。规划侧见 [dddnav_global_planner](../dddnav_global_planner/) · [dddnav_local_planner](../dddnav_local_planner/)。

<table>
  <tr>
    <td width="33%"><img src="https://github.com/dfl-rlab/dddnav_documentation_materials/blob/main/perception_3d/perception_3d_global_plan.gif"/>全局规划</td>
    <td width="33%"><img src="https://github.com/dfl-rlab/dddnav_documentation_materials/blob/main/perception_3d/marking_tracking_clearing.gif"/>标记/清除</td>
    <td width="33%"><img src="https://github.com/dfl-rlab/dddnav_documentation_materials/blob/main/perception_3d/speed_limit_zone.png"/>限速/禁入</td>
  </tr>
</table>

**传感器：** 多线旋转雷达 · 深度相机 · 扫描雷达（Mid360 / Unitree L1 等）  
**图层：** static · speed limit · no-entry

---

## Demo 流程（可选 Docker）

`REPO` = 含 `src/`、`dddnav_docker/` 的目录。

```bash
cd /path/to/REPO/dddnav_docker/docker_file && ./build.bash
cd /path/to/REPO/dddnav_docker && ./run_demo.bash   # 工作区挂到容器 /root/dddnav_navigation
```

在容器或本机已编译环境中：

```bash
cd /path/to/REPO/src/dddnav_perception_3d && ./download_files.bash   # 按 demo 下 bag
cd /path/to/REPO && source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

| Demo | Launch |
|------|--------|
| 多线雷达 (Leishen C16) | `ros2 launch perception_3d multilayer_spinning_lidar_3d_ros_launch.py` |
| 双深度相机 | `ros2 launch perception_3d multi_depth_camera_3d_ros_launch.py` |
| 扫描雷达 (Unitree G4) | `ros2 launch perception_3d scanning_lidar_3d_ros_launch.py` |

Launch 后约 3s 自动播 bag；bag 默认在 **`~/dddnav_bags/...`**（与根 README / docker 说明一致）。

<p align="center"><img src="https://github.com/dfl-rlab/dddnav_documentation_materials/blob/main/perception_3d/multilayer_lidar_demo.gif" width="640" height="400"/></p>
<p align="center"><img src="https://github.com/dfl-rlab/dddnav_documentation_materials/blob/main/perception_3d/multi_depth_camera_demo.gif" width="640" height="400"/></p>
<p align="center"><img src="https://github.com/dfl-rlab/dddnav_documentation_materials/blob/main/perception_3d/scanning_lidar_demo.gif" width="640" height="400"/></p>

## 限速 / 禁入 YAML

运行时相对 **`perception_3d` 的 share** 解析路径：[speed_limit_layer.yaml](config/speed_limit_layer.yaml) · [no_entry_layer.yaml](config/no_entry_layer.yaml)

## Zone 编辑

```bash
ros2 launch perception_3d zone_editor_utils.launch
```

[![zone editor](https://github.com/dfl-rlab/dddnav_documentation_materials/blob/main/perception_3d/point_cloud_editor.png)](https://youtu.be/DHgzRD4HrjU)
