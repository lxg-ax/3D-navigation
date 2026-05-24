# dddnav_p2p_move_base

ROS 包名 **`p2p_move_base`**。3D 点到点导航的总调度节点：FSM + cmd_vel 输出 + 全局/局部规划协同 + recovery 触发。

## 原理

内部是一个有限状态机（`p2p_fsm.cpp`），主要状态：

```
IDLE → PLANNING → CONTROLLING → (WAITING | RECOVERING) → PLANNING / DONE
```

* **PLANNING**：调 `global_planner` 的 action 拿 3D 路径
* **CONTROLLING**：把路径喂给 `local_planner`，订其输出的速度直接 `publish` 到 `cmd_vel`（或 `cmd_vel_stamped`）
* **WAITING**：路径被障碍物挡住时停车等 `waiting_patience` 秒，到点了重新规划
* **RECOVERING**：调用 `recovery_behaviors`（默认原地转），可被 cancel
* 旋转 shim 思路类似 [nav2_rotation_shim_controller](https://github.com/ros-navigation/navigation2/tree/main/nav2_rotation_shim_controller)：起步先把朝向对齐路径

## 在系统中的角色

整个导航的"司机"。bringup 起来后就是它在调度规划与控制：

```
RViz / 业务节点 ─► /goal ─► p2p_move_base
                               │
                               ├── action ──► global_planner
                               ├── topic  ──► local_planner ──► cmd_vel
                               └── recovery_behaviors（卡住时）
```

## 主要参数

`dddnav_bringup/config/nav/base.yaml` 的 `p2p_move_base` 段：

| Key | 含义 |
|-----|------|
| `controller_frequency` | 控制环频率，默认 10Hz |
| `planner_patience` | 全局规划失败后等多久再试 |
| `waiting_patience` | 路径被挡时等多久才放弃当前路径重规划 |
| `oscillation_distance / angle / patience` | 抖动检测，触发 recovery |
| `no_plan_retry_num` | 连续规划失败到多少次进 recovery |

## TODO

* recovery 现在只挂 RotateInPlace；后退、清局部图、寻找空间方向等已有插槽，待按业务挂上
* FSM 是 cpp 硬编码，长期看可以迁到 BehaviorTree.CPP

## 老 demo

`go2_localization.launch` 还能跑（Go2 + bag），主线 Mid360 用 `ros2 launch dddnav_bringup localization.launch.py`。
