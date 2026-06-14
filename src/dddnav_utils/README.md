# dddnav_utils

工具节点合集：传感器桥接 / 地图保存 / 全局重定位 / 健康监控。多数节点都从 `dddnav_bringup` 的 launch 间接拉起。

## 节点

| 可执行 | 语言 | 作用 |
|--------|------|------|
| `livox_pc2_to_liosam` | C++ | Livox PointCloud2 → LIO-SAM 用的 `VelodynePointXYZIRT` + MCL 用的 `PointXYZI` 双发布。**Publisher 用 RELIABLE QoS** 与 LIO-SAM `imageProjection.qos_lidar=RELIABLE` 匹配（不兼容时整条后端管道空转但节点都活着，参见下方 Bridge QoS 段）。 |
| `std_global_init` | C++ | STD 全局重定位。启动期对每帧雷达生成三角描述符查表给 `/initial_3d_pose`；运行期低频 watchdog 持续比对 STD 候选与 MCL 当前位置，连续不一致再发种子触发 MCL 重启。免操作员点 RViz、被搬运 / 走错楼层可自动恢复。STD 是 6-DoF 旋转/平移不变描述子（[HKU-MaRS-Lab/STD](https://github.com/hku-mars/STD)），相比 Scan Context 不依赖列移位恢复 yaw、候选位姿带 plane-ICP 验证。 |
| `liosam_to_posegraph.py` | Python | 订 LIO-SAM 关键帧，写 binary PCD + pose graph 到 `dddnav_bringup/map/`。 |
| `fastlio_to_posegraph.py` | Python | 同上，但走纯 FAST-LIO 不挂 LIO-SAM 的场景。 |
| `slam_health_monitor.py` | Python | 看 `/Odometry` 和 LIO-SAM odom 速率、TF 边 liveness、`map→odom` 跳变、`/odom_filtered` cov trace，发 `/diagnostics`。 |
| `nav_perf_monitor.py` | Python | 看 `cmd_vel` / 全局规划路径速率、FAST-LIO 残差与 feats，发 `/diagnostics`。 |
| `dddnav_preflight.py` | Python | 启动期自检：QoS 兼容性 / TF 单一所有者 / 跨节点参数一致性。warmup 后跑一次、recheck 周期复查，ERROR 推到 `/diagnostics`。 |
| `save_map_on_exit.py` | Python | mapping launch 的 `auto_save_on_exit` 钩子。Ctrl-C 时按顺序 `pose_graph → /lio_sam/save_map`，比 bash 链稳。 |

## 原理要点

### STD 全局初始化 + 在线 watchdog

* 建图阶段每个关键帧算 STD（平面分割 → 角点 → 三角描述子），body 帧落盘 `<map>/lio_sam/std_db.bin`
* **启动期**：当前 LiDAR 帧三边长 hash 检索 → 候选 plane-ICP 验证。`std_score_threshold` 把关 + 连续 `min_consensus_frames` 一致才发，过滤偶然误匹配
* **运行期 watchdog**：以 `watchdog_check_hz`（默认 2 Hz）评估，**全部满足才发** `/initial_3d_pose`：
  * `score ≥ relocate_min_score`
  * 候选关键帧距 MCL 当前关键帧 ≥ `relocate_min_dist_from_live_m`（避免 MCL 自洽时无意义重发）
  * 候选距 odom 先验位置 ≤ `relocate_max_jump_m`、yaw 差 ≤ `relocate_max_jump_yaw`
  * 连续 `relocate_consensus` 次都满足
  * 距上次发布 ≥ `relocate_holdoff_sec`（冷却）
* 候选位姿 `T_world_kf · T_kf_query`，roll/pitch 用 STD 自身估计而非关键帧复制
* 同时解决长走廊误匹配（spatial gate + plane-ICP score）和被搬运 MCL 卡死（持续不一致触发重启）

### Preflight 自检

启动期暴露三类"看起来都活着实际全空转"的故障：

1. **QoS 兼容性**：扫高影响话题的 publisher/subscriber，按 DDS 标准表判定（BE↔RELIABLE 这种隐形丢消息红灯）
2. **TF 单一所有者**：拉 `/tf` publisher 集合对照运行模式（localization 下 `lio_sam_mapOptimization` 或 `mcl_3dl` 出现就报错）
3. **跨节点参数一致性**：`mcl_3dl.publish_tf=false`、`num_particles_min ≤ ... ≤ max`、`sub_map_search_radius` 在 0.3~1.5×`lidar_max_range`、`pose_fusion.publish_tf` / `lio_sam.publishTF` 与 mode 对齐

绝不主动杀 launch，由人 / 上层据诊断决定。节点没起来的检查项静默跳过下个 tick 再试。

| Key | 默认 | 含义 |
|-----|------|------|
| `mode` | `"localization"` | 决定 TF / publish_tf 期望集 |
| `warmup_sec` | 5.0 | 首次检查延迟 |
| `recheck_sec` | 30.0 | 周期复查间隔 |
| `qos_topics` | 9 个高影响话题 | 要查 QoS 的话题 |
| `tf_unique_edges` | `["map->odom", "odom->base_link"]` | 期望唯一 owner 的边 |
| `lidar_max_range` | 70.0 m | 校验 `sub_map_search_radius` |

### Health / perf 阈值

* `slam_health_monitor`：`OK` (≤ 0.5 s) → `WARN` (≤ 2 s) → `ERROR` (> 2 s)。建图模式 launch 放宽到 `ok_timeout=1.0 s / fail_timeout=3.0 s`（LIO-SAM 单次优化偶发 100~150 ms 会触发默认阈值），同时 `filtered_odom_topic` 置空（pose_fusion 不在建图链）
* `nav_perf_monitor`：慢于一半 WARN、五分之一 ERROR。FAST-LIO 残差走 `lio_residual_warn / error`

两节点只对 WARN/ERROR 出日志，OK 状态靠 `/diagnostics` 流式呈现。

### Bridge QoS 兼容性（坑点）

`livox_pc2_to_liosam` 两个 publisher 一律 **RELIABLE**。改成 BEST_EFFORT 时 LIO-SAM `imageProjection.qos_lidar=RELIABLE` 不兼容直接收不到点云，整条后端（`cloud_deskewed` / `feature/cloud_info` / `mapping/odometry` / `map→odom` TF）全程空转但节点都活着。改 QoS 时保留 RELIABLE。

### Adaptive save sequence

LIO-SAM 的 `save_map` service 会 `rm -r` 自己的目录，必须先存 pose graph 再存 LIO-SAM。`save_map_on_exit.py` 用 rclpy 客户端做这件事，比 bash 串 `ros2 service call` 在 launch tear-down 里更稳。

## 在系统中的角色

* 建图链：Livox driver → `livox_pc2_to_liosam` → FAST-LIO + LIO-SAM → `liosam_to_posegraph` → `save_map_on_exit`
* 定位链：`dddnav_preflight` → `std_global_init`（启动 + watchdog）→ MCL 收敛 → `slam_health_monitor` + `nav_perf_monitor`

## STD 参数（默认通常够用）

| Key | 默认 | 含义 |
|-----|------|------|
| `std_score_threshold` | 0.50 | 启动期 plane-ICP 最低 score（0.3~0.7 常用） |
| `min_consensus_frames` | 3 | 启动期连续命中同一索引才确认 |
| `min_points` | 1500 | 输入点过少跳过查询 |
| `init_cov_xy / z / yaw / rp` | 1.0 / 0.25 / 0.09 / 0.04 | 发出去的协方差 |
| `enable_watchdog` | true | 关掉退化为"启动期一次性" |
| `watchdog_check_hz` | 2.0 | watchdog 评估频率 |
| `watchdog_warmup_sec` | 8.0 | 首次发布后 watchdog 上线延迟 |
| `relocate_min_score` | 0.55 | watchdog 最低 STD score（略高于启动期） |
| `relocate_min_dist_from_live_m` | 4.0 | 候选距 MCL 当前关键帧最低距离 |
| `relocate_max_jump_m / _yaw` | 6.0 / 1.05 rad | 候选与 odom 先验的位置/朝向上限 |
| `relocate_consensus` | 4 | 连续 tick 一致数 |
| `relocate_holdoff_sec` | 10.0 | 发布后冷却 |

无 `std_db.bin` 时节点直接退出，`runtime.yaml.initial_pose` 兜底。
