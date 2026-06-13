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

## generator + 评分组合：argmin vs MPPI softmax

`getBestTrajectory` 把"挑最优"那一步委托给 generator 的 `combineByScores` 钩子：

* 默认实现是 **argmin**（DDSimple / RotateInplace / Omni 都吃这个），跟以前一样选 cost 最低那条
* `MPPIDifferentialDriveTheory` 重写了它：`w_i = exp(-(S_i - min S)/λ)` 做 softmax，对样本的控制序列加权平均得 `U*`，再 rollout 一次作为本周期输出，并把 `U*` 左移一格作为下周期 nominal warm start

MPPI 关键参数（`differential_drive_mppi.mppi.*`，示例见
`dddnav_bringup/config/reality/tuning/example_mppi_overlay.yaml`）：

| Key | 含义 | 默认 |
|-----|------|------|
| `num_samples` | 每周期采样数 N | 64 |
| `lambda` | softmax 温度。小 → 接近 argmin，大 → 平滑 | 1.0 |
| `sigma_v` / `sigma_w` | 控制扰动高斯标准差（m/s, rad/s） | 0.2 / 0.4 |
| `reset_nominal_each_cycle` | 关掉 warm start，每周期从 0 重启 nominal | false |
| `min_valid_fraction` | 通过 critic 的样本比例低于这个值就降级 argmin | 0.1 |

`Trajectory.controls_` 是新加的可选字段，存每条 rollout 的逐步 (v, ω)。
DDSimple / RotateInplace / Omni 留空，只有 MPPI 在用。

## 在系统中的角色

`p2p_move_base` 拿到全局路径后驱动 `local_planner` 输出 `cmd_vel`；卡住时按配置触发 `recovery_behaviors`（原地转 → 重新规划）。

## 主要参数

`dddnav_bringup/config/nav_base.yaml` 是公共层。常调：

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
