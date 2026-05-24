# dddnav_global_planner

ROS 包名 **`global_planner`**。在 `perception_3d` 输出的地面图（ground graph）上做全局路径规划。

总览：[根 README](../../README.md)。规划下游对接 [`dddnav_local_planner`](../dddnav_local_planner/) 与 [`dddnav_p2p_move_base`](../dddnav_p2p_move_base/)。

## 原理

* 输入：`perception_3d` 把地面点云抽成节点，邻居按半径 + 高度差连边形成 3D ground graph
* 算法：节点上的 A\* 搜索；`turning_weight` 抑制 zig-zag，`a_star_expanding_radius` 控制每次扩展的邻居半径
* 静态地图模式（`use_pre_graph: true`）会预构图，每次规划只查图，省掉在线展开邻居
* 输出：`nav_msgs/Path`，经 `p2p_global_plan_manager` 按 `global_plan_query_frequency` 周期重查

## 在系统中的角色

定位流程下，`p2p_move_base` 收到目标 → 调 `global_planner` 的 action 拿全局路径 → 交给 `local_planner` 跟随。回环或重定位让 `map→odom` 跳变时，`p2p_global_plan_manager` 5Hz 重查路径自动刷新。

## 主要参数

`dddnav_bringup/config/nav/base.yaml` 与各 profile overlay 里的 `global_planner` 段：

| Key | 说明 |
|-----|------|
| `turning_weight` | 转弯代价权重，越大路径越直 |
| `a_star_expanding_radius` | A\* 单次扩展邻居半径（米） |
| `use_pre_graph` | 静态地图下预构图加速规划 |

## Demo

```bash
ros2 launch global_planner path_planning_on_static_layer.launch
```

RViz 里 **Publish Point** 发目标。
