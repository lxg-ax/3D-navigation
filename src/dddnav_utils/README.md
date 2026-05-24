# dddnav_utils

工具节点合集：传感器桥接 / 地图保存 / 全局重定位 / 健康监控。多数节点都从 `dddnav_bringup` 的 launch 间接拉起。

## 节点

| 可执行 | 语言 | 作用 |
|--------|------|------|
| `livox_pc2_to_liosam` | C++ | Livox PointCloud2 → LIO-SAM 用的 `VelodynePointXYZIRT` + MCL 用的 `PointXYZI` 双发布。10Hz 数据路径，C++ 是为了避开 Python GIL 抖动。 |
| `sc_global_init` | C++ | Scan Context 全局重定位。加载建图阶段写的 `sc_db.bin` + `poses.pcd`，对第一帧雷达查表，连续 N 帧一致就发 `/initial_3d_pose`。免操作员点 RViz 初始位姿。 |
| `liosam_to_posegraph.py` | Python | 订 LIO-SAM 关键帧，写 binary PCD + pose graph 到 `dddnav_bringup/map/`。 |
| `fastlio_to_posegraph.py` | Python | 同上，但走纯 FAST-LIO 不挂 LIO-SAM 的场景。 |
| `slam_health_monitor.py` | Python | 看 `/Odometry` 和 LIO-SAM odom 速率、TF 边 liveness、`map→odom` 跳变、`/odom_filtered` cov trace，发 `diagnostic_msgs/DiagnosticArray`。 |
| `nav_perf_monitor.py` | Python | 看 `cmd_vel` / 全局规划路径速率、FAST-LIO 残差与 feats，同样发 `/diagnostics`。 |
| `save_map_on_exit.py` | Python | mapping launch 的 `auto_save_on_exit` 钩子。Ctrl-C 时按顺序 `pose_graph → /lio_sam/save_map`，比 bash 链稳。 |

## 原理要点

### Scan Context 全局初始化

* 描述符（20×60 的极坐标俯视图）在建图阶段每个关键帧算一遍，body 帧描述符落盘 `sc_db.bin`
* 启动时对当前 LiDAR 帧算同样描述符，KD-Tree 找 ring key 候选 → 列移位余弦距离精排
* 阈值 `sc_dist_threshold` + 连续 `min_consensus_frames` 一致才发，过滤偶然误匹配
* 列移位顺便给出 yaw 修正，roll/pitch 直接复制关键帧位姿（地面机器人这两个量小）

### Adaptive save sequence

LIO-SAM 的 `save_map` service 会 `rm -r` 自己的目录，必须先存 pose graph 再存 LIO-SAM；老方案用 bash 串两个 `ros2 service call` 在 launch tear-down 里偶尔挂。`save_map_on_exit.py` 用 rclpy 客户端做同样的事，环境变量被改也不受影响。

### Health / perf 阈值

* `slam_health_monitor`：`OK` (≤ 0.5 s) → `WARN` (≤ 2 s) → `ERROR` (> 2 s) per topic / TF 边
* `nav_perf_monitor`：阈值跟着 100Hz odom + 5Hz planner 的预期速率走，慢于一半进 WARN，慢于五分之一进 ERROR；FAST-LIO 残差走自己的 `lio_residual_warn / error`

两个节点都只对 WARN/ERROR 出文字日志，OK 状态靠 `/diagnostics` 流式呈现。

## 在系统中的角色

* 建图链：Livox driver → `livox_pc2_to_liosam` → FAST-LIO + LIO-SAM → `liosam_to_posegraph` 写 pose graph → `save_map_on_exit` 落盘
* 定位链：`sc_global_init` 给初始位姿 → MCL 收敛 → `slam_health_monitor` + `nav_perf_monitor` 在线监控

## SC 全局初始化参数

通常默认就够，要改在 launch 里通过 `sc_init_yaml` 或者 CLI `-p` 传：

| Key | 默认 | 含义 |
|-----|------|------|
| `sc_dist_threshold` | 0.30 | 余弦距离阈值，越小越严格 |
| `min_consensus_frames` | 3 | 连续命中同一索引才确认 |
| `min_points` | 1500 | 输入点过少跳过查询 |
| `init_cov_xy / z / yaw / rp` | 1.0 / 0.25 / 0.09 / 0.04 | 发出去给 MCL 的协方差 |

无 `sc_db.bin` 时节点直接退出，`runtime.yaml.initial_pose` 仍生效。
