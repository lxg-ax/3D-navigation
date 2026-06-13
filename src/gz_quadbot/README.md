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

## 仿真导航（Mid360 主线 + Gazebo VLP-16）

`dddnav_bringup` 提供三个仿真专用 launch，自动起 Gazebo Go2 + 整套 SLAM/定位/规划栈，全程 `use_sim_time:=true`：

| Launch | 用途 |
|------|------|
| `sim_mapping.launch.py` | Gazebo + FAST-LIO + LIO-SAM 后端，纯建图 |
| `sim_mapping_nav.launch.py` | 上行 + 全局 / 局部规划，边建边跑 |
| `sim_localization.launch.py` | Gazebo + FAST-LIO + MCL + ESKF + 规划，需先有位姿图 |

```bash
# 1. 先用仿真建一张图（自动落到 share/dddnav_bringup/map/）
ros2 launch dddnav_bringup sim_mapping.launch.py auto_save_on_exit:=true
# 操作 cmd_vel 走一圈，Ctrl-C 出图

# 2. 用刚建的图启动定位 + 导航
ros2 launch dddnav_bringup sim_localization.launch.py
```

仿真专用配置：

| 文件 | 作用 |
|------|------|
| `dddnav_bringup/config/simulation/fastlio_velodyne_sim.yaml` | FAST-LIO 适配 VLP-16，订阅 `/velodyne_points` + `/imu/data` |
| `dddnav_bringup/config/simulation/lio_sam_velodyne_sim.yaml` | LIO-SAM 后端，N_SCAN=16、轻噪声 IMU |
| `dddnav_bringup/config/simulation/nav/sim_velodyne_mapping.yaml` | mapping_nav 的 perception/planner overlay |
| `dddnav_bringup/config/simulation/nav/sim_velodyne_localization.yaml` | localization 的 perception/planner overlay |

仿真和真机走两条独立链路：仿真无 Livox 桥接，FAST-LIO 直接吃 `/velodyne_points`；mcl_feature 与 std_global_init 在仿真里改吃 `/cloud_registered_body`。其余节点接线与真机一致。

## 致谢

* [CHAMP](https://github.com/chvmp/champ) 和 [chvmp/robots](https://github.com/chvmp/robots) — 步态控制框架与配置生成器
* [unitree-go2-ros2](https://github.com/anujjain-dev/unitree-go2-ros2) — Go2 ROS 2 接入
* [Unitree](https://github.com/unitreerobotics/unitree_ros) — Go2 URDF / mesh
