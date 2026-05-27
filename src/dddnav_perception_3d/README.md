# dddnav_perception_3d

ROS 包名 **`perception_3d`**。3D 代价地图，对应 Nav2 里 `nav2_costmap_2d` 的角色，但走的是点云。

## 原理

* 把"地面 + 障碍"建模成 3D ground graph：地面点连成图（节点 + 邻居边），障碍点投到对应 voxel 标记
* 多个 layer 叠加（plugin 体系），每层维护自己的标记 / 清除规则
* 输出两份代价图：`perception_3d_local`（局部，给 local planner 做碰撞）和 `perception_3d_global`（全局，给 global planner 用图搜索）

## 内置图层

| 插件 | 作用 |
|------|------|
| `StaticLayer` | 加载位姿图存的静态点云（`mapcloud` / `mapground` 或 mapping 模式下的 `lego_loam_map`），构 ground graph |
| `MultiLayerSpinningLidar` | 当前雷达点云做标记 / 清除（按扇区切片，原本写给旋转雷达） |
| `DepthCameraLayer` | 深度相机 / 语义点云接入，补 LiDAR 近距离盲区 |
| `PathBlockedStrategy` | 检测路径前方是否被堵 |
| `SpeedLimitLayer` / `NoEntryLayer` | 限速区 / 禁入区，外部 PCD 加载 |
| `ClusterMarking` | 点云聚类标记（动态物体雏形） |

## 在系统中的角色

```
LiDAR / 深度相机 / 语义点云 ─► perception_3d_local  ─► local_planner
                                                  
位姿图 + 子图 ─────────────► perception_3d_global ─► global_planner
```

每个 layer 在 `dddnav_bringup/config/reality/nav/<profile>.yaml` 里通过 `plugins:` 列表挂载 + 各自参数。

## 主要参数

| Key | 含义 |
|-----|------|
| `plugins` | 当前代价图挂哪些层 |
| `inscribed_radius` / `inflation_radius` | 内切圆 / 膨胀半径 |
| `inflation_descending_rate` | 代价随距离衰减率 |
| `max_obstacle_distance` | 障碍点保留的最大距离 |
| 各插件子段 | 插件自己的细参数 |

## 限速区 / 禁入区

YAML 在 [`config/speed_limit_layer.yaml`](config/speed_limit_layer.yaml) / [`config/no_entry_layer.yaml`](config/no_entry_layer.yaml)，运行时用 PCD 文件圈定区域。

## Zone 编辑器

```bash
ros2 launch perception_3d zone_editor_utils.launch
```

## 已知点（Mid360 适配）

`MultiLayerSpinningLidar` 默认是给旋转雷达写的扇区切片；Mid360 的非重复扫描下 `vertical_FOV_*` + `scan_effective_*` 这套切片不准。短期内通过 `perception_window_size` / `segmentation_ignore_ratio` 调整能用，长期建议给 Mid360 写一个基于时空积累的专用动态层。
