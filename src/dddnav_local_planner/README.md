# dddnav_local_planner

Metapackage，包含五个子包：

| 子包 | 作用 |
|------|------|
| `local_planner` | 局部规划主节点：读全局路径 + 当前位姿 + 局部代价图，按 critic 评分选最优速度命令 |
| `trajectory_generators` | 轨迹族生成器（差速 simple、原地转、最短角原地转，支持新增） |
| `mpc_critics` | 评分插件（碰撞、贴合路径、纯跟踪、对齐全局路径、偏好原地最短角等） |
| `recovery_behaviors` | 恢复行为插件（当前主要 RotateInPlace） |
| `base_trajectory` | 轨迹通用基类与几何工具（cuboid 碰撞、采样） |

## 原理

* **采样型 DWA + cuboid 碰撞**：每周期由 `trajectory_generators` 按 `(vx, ω)` 网格采几十条候选轨迹（默认 5×10=50 条）
* 每条轨迹按 `sim_granularity` 离散成点，对每点用机器人 cuboid（8 顶点）查 `perception_3d` 看是否碰撞
* `mpc_critics` 对每条幸存轨迹打分（多个 critic 加权和），得分最高的执行
* 名字叫 mpc_critics 但实质是 **采样 + 评分**，不是连续优化的 MPC

## 在系统中的角色

`p2p_move_base` 拿到全局路径后驱动 `local_planner` 输出 `cmd_vel`；卡住时按配置触发 `recovery_behaviors`（原地转 → 重新规划）。

## 主要参数

`dddnav_bringup/config/nav/base.yaml` 是公共层。常调：

| Key | 含义 |
|-----|------|
| `local_planner.controller_frequency` | 控制周期，默认 10Hz |
| `differential_drive_simple.{max_vel_x, max_vel_theta, sim_time, sim_granularity}` | 速度极限、前向预测时间、轨迹采样粒度 |
| `cuboid.{flb, frb, ...}` | 8 顶点底盘外形 |
| `mpc_critics.{collision, pure_pursuit, toward_global_plan, ...}.weight` | 各 critic 权重 |

## Demo

```bash
ros2 launch local_planner local_planner_play_ground.launch
```
