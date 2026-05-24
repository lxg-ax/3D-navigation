# gz_quadbot（Gazebo Go2）

Go2 + Gazebo 仿真环境，基于 [CHAMP](https://github.com/chvmp/champ) 与 [unitree-go2-ros2](https://github.com/anujjain-dev/unitree-go2-ros2) 二次集成。给本仓库的导航栈提供仿真验证场，也保留了 Go2 真机演示的入口。

真机 3D 导航请用 `dddnav_bringup`（见 [根 README](../../README.md)），本包只用于仿真。

## 子模块

| 路径 | 来源 | 作用 |
|------|------|------|
| `champ/` | 上游 CHAMP | 四足控制框架（步态、足端轨迹、impedance 控制） |
| `robots/` | 上游 chvmp/robots | CHAMP setup-assistant 生成的多种四足配置包 |
| `gz_go2_demo_world/` | 本仓库 | 给 Mid360 / 位姿图演示打包的世界 / 地图 |

## 默认 World 与 Map

**Worlds：** `robots/configs/go2_config/worlds/`，安装到 `$(ros2 pkg prefix go2_config)/share/go2_config/worlds/`

| Launch | 默认 world |
|--------|------------|
| [`gazebo.launch.py`](robots/configs/go2_config/launch/gazebo.launch.py) | `default.world` |
| [`gazebo_velodyne.launch.py`](robots/configs/go2_config/launch/gazebo_velodyne.launch.py) | `slope_with_pillar_2.world` |
| [`gz_lidar_odom.launch.py`](robots/configs/go2_config/launch/gz_lidar_odom.launch.py) | `slope_with_pillar_2.world` |

覆盖：

```bash
ros2 launch ... world:=$(ros2 pkg prefix go2_config)/share/go2_config/worlds/playground.world
```

**Nav2 2D 栅格地图：** `robots/configs/go2_config/maps/`（`map.yaml` ↔ `map.pgm` 等，YAML 里写的相对路径不要拆散）。

`champ_config/launch/navigate.launch.py` 默认指向 CHAMP 自带地图；要在 Gazebo Go2 里跑得显式传 go2 的图：

```bash
ros2 launch champ_config navigate.launch.py \
  sim:=true \
  map:=$(ros2 pkg prefix go2_config)/share/go2_config/maps/playground.yaml \
  params_file:=$(ros2 pkg prefix go2_config)/share/go2_config/config/autonomy/navigation.yaml
```

## 与 MCL 位姿图的关系

标准位姿图数据放 **`src/dddnav_bringup/map/`**。Go2 老 demo `ros2 launch p2p_move_base go2_localization.launch` 在 launch 里把 `sub_maps.pose_graph_dir` 指向那个目录；Mid360 主线 `ros2 launch dddnav_bringup localization.launch.py` 走同一份位姿图。

## 致谢

* [CHAMP](https://github.com/chvmp/champ) 和 [chvmp/robots](https://github.com/chvmp/robots) — 步态控制框架与配置生成器
* [unitree-go2-ros2](https://github.com/anujjain-dev/unitree-go2-ros2) — Go2 ROS 2 接入
* [Unitree](https://github.com/unitreerobotics/unitree_ros) — Go2 URDF / mesh
