# Gazebo Go2 (gz_quadbot)

Go2 + Gazebo，基于 [CHAMP](https://github.com/chvmp/champ) 与 [unitree-go2-ros2](https://github.com/anujjain-dev/unitree-go2-ros2)。真机 3D 导航用 **`dddnav_bringup`**，见 [根 README](../../README.md)。此处不放外链 GIF。

## World / map 路径

**World：** `robots/configs/go2_config/worlds/` → 安装 `$(ros2 pkg prefix go2_config)/share/go2_config/worlds/`

| Launch | 默认 world |
|--------|------------|
| [gazebo.launch.py](robots/configs/go2_config/launch/gazebo.launch.py) | `default.world` |
| [gazebo_velodyne.launch.py](robots/configs/go2_config/launch/gazebo_velodyne.launch.py) | `slope_with_pillar_2.world` |
| [gz_lidar_odom.launch.py](robots/configs/go2_config/launch/gz_lidar_odom.launch.py) | `slope_with_pillar_2.world` |

覆盖：`world:=$(ros2 pkg prefix go2_config)/share/go2_config/worlds/playground.world` 等。

**2D Nav2 栅格：** `robots/configs/go2_config/maps/`（`map.yaml`→`map.pgm`，`playground.yaml`→`playground.pgm`，YAML 里相对路径别拆散）。

`champ_config/launch/navigate.launch.py` 默认 map 指向 **champ** 自带图；若 Gazebo 跑 Go2 要用 go2 的图，需显式传参，例如：

```bash
ros2 launch champ_config navigate.launch.py \
  sim:=true \
  map:=$(ros2 pkg prefix go2_config)/share/go2_config/maps/playground.yaml \
  params_file:=$(ros2 pkg prefix go2_config)/share/go2_config/config/autonomy/navigation.yaml
```

CHAMP 通用图：`champ/champ_config/maps/`。

## 和 MCL 位姿图

标准数据在 **`src/dddnav_bringup/map/`**。历史说明：[gz_go2_demo_world/README.md](gz_go2_demo_world/README.md)。  
`ros2 launch p2p_move_base go2_localization.launch` → Python 里设 **`sub_maps.pose_graph_dir`** 指上述目录；Mid360 用 **`ros2 launch dddnav_bringup localization.launch.py`** 同理。

## Credits

[unitree-go2-ros2](https://github.com/anujjain-dev/unitree-go2-ros2)、[Unitree](https://github.com/unitreerobotics/unitree_ros)、[CHAMP](https://github.com/chvmp/champ)、[chvmp/robots](https://github.com/chvmp/robots)。
