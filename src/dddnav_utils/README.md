# dddnav_utils

工具节点合集：传感器桥接 / 地图保存 / 全局重定位 / 健康监控。多数节点都从 `dddnav_bringup` 的 launch 间接拉起。

## 节点

| 可执行 | 语言 | 作用 |
|--------|------|------|
| `livox_pc2_to_liosam` | C++ | Livox PointCloud2 → LIO-SAM 用的 `VelodynePointXYZIRT` + MCL 用的 `PointXYZI` 双发布。10Hz 数据路径，C++ 是为了避开 Python GIL 抖动。**Publisher 用 RELIABLE QoS** 与 LIO-SAM `imageProjection` 的 `qos_lidar`（RELIABLE）匹配，否则 DDS QoS 不兼容会让 LIO-SAM 后端永远收不到点云。 |
| `std_global_init` | C++ | STD（Stable Triangle Descriptor）全局重定位。启动期对每帧雷达生成三角描述符查表给 `/initial_3d_pose`；运行期低频比对当前帧 STD 检索结果与 MCL 当前关键帧位置，连续不一致且候选靠近 `/odom_filtered` 先验时再次发种子触发 MCL 重启。免操作员点 RViz 初始位姿，也能从被搬运 / 走错楼层中自动恢复。STD 是 6-DoF 旋转/平移不变描述子（来源 [HKU-MaRS-Lab/STD](https://github.com/hku-mars/STD)），相比 Scan Context 不再依赖列移位恢复 yaw，候选位姿带 plane-ICP 验证。 |
| `liosam_to_posegraph.py` | Python | 订 LIO-SAM 关键帧，写 binary PCD + pose graph 到 `dddnav_bringup/map/`。 |
| `fastlio_to_posegraph.py` | Python | 同上，但走纯 FAST-LIO 不挂 LIO-SAM 的场景。 |
| `slam_health_monitor.py` | Python | 看 `/Odometry` 和 LIO-SAM odom 速率、TF 边 liveness、`map→odom` 跳变、`/odom_filtered` cov trace，发 `diagnostic_msgs/DiagnosticArray`。 |
| `nav_perf_monitor.py` | Python | 看 `cmd_vel` / 全局规划路径速率、FAST-LIO 残差与 feats，同样发 `/diagnostics`。 |
| `dddnav_preflight.py` | Python | 启动期自检：扫高影响话题的 publisher/subscriber QoS 兼容性、查 `/tf` publisher 集合是否与运行模式相符、抓 `mcl_3dl` / `pose_fusion` / `lio_sam_*` 关键参数做交叉一致性校验。warmup_sec 后跑一次、recheck_sec 周期复查，把 ERROR 推到 `/diagnostics`。 |
| `save_map_on_exit.py` | Python | mapping launch 的 `auto_save_on_exit` 钩子。Ctrl-C 时按顺序 `pose_graph → /lio_sam/save_map`，比 bash 链稳。 |

## 原理要点

### STD 全局初始化 + 在线重定位巡检

* 关键帧点云做平面分割 → 角点 → 三角描述符 (STD) 在建图阶段每个关键帧算一遍，body 帧描述符落盘到 `<map>/lio_sam/std_db.bin`（与平面/角点云一并存档）
* **启动期**：对当前 LiDAR 帧生成 STD 三角描述子，按三边长 hash 检索 → 候选关键帧上做 plane-ICP 验证。score 阈值 `std_score_threshold`（越大越严）+ 连续 `min_consensus_frames` 一致才发，过滤偶然误匹配
* **运行期（watchdog）**：节点持续订 `/odom_filtered`。在 `watchdog_warmup_sec` 后以 `watchdog_check_hz`（默认 2 Hz）评估每帧
  1. 全 DB 上查 STD 最佳候选（cand_idx, score, T_kf_query）
  2. 仅当 `score ≥ relocate_min_score` **且** 候选关键帧到 MCL 当前最近关键帧距离 ≥ `relocate_min_dist_from_live_m`（避免和 MCL 自洽时无意义重发） **且** 候选到 odom 先验位置 ≤ `relocate_max_jump_m` **且** 候选 yaw 与先验 yaw 之差 ≤ `relocate_max_jump_yaw` **且** 连续 `relocate_consensus` 次都满足，才再发 `/initial_3d_pose`
  3. `relocate_holdoff_sec` 是发布后的冷却时间，给 MCL 收敛
* STD 直接给 6-DoF 相对位姿，世界系候选位姿 = `T_world_kf · T_kf_query`，roll/pitch 走 STD 自身估计而不是关键帧复制
* 这套门控同时解决两类问题：长走廊 / 对称楼道描述子误匹配（spatial gate + plane-ICP score 双重过滤），被搬运 / 漂错楼层 MCL 卡死（持续不一致触发重启）

### Adaptive save sequence

LIO-SAM 的 `save_map` service 会 `rm -r` 自己的目录，必须先存 pose graph 再存 LIO-SAM。老方案用 bash 串两个 `ros2 service call` 在 launch tear-down 里偶尔挂。`save_map_on_exit.py` 改用 rclpy 客户端做同样的事，环境变量被改也不受影响。

### Preflight 自检（`dddnav_preflight.py`）

启动期暴露三类"看起来都活着实际全空转"的故障，5 秒内推 ERROR 到 `/diagnostics`：

1. **QoS 兼容性**：用 `get_publishers_info_by_topic` / `get_subscriptions_info_by_topic` 逐条对比关键边（`/livox/lidar_liosam*`、`/Odometry`、`/odom_filtered`、`/laser_cloud_*`），按 DDS 标准表判定（RELIABLE→BEST_EFFORT 通过、BEST_EFFORT→RELIABLE 不通过；TRANSIENT_LOCAL→VOLATILE 通过、反之不通过）。BE↔RELIABLE 这种隐形丢消息直接红灯。
2. **TF 单一所有者**：拉 `/tf` 上所有 publisher 的 node_name，对照运行模式的允许集合。`localization` 模式下 `lio_sam_mapOptimization` 或 `mcl_3dl` 出现在 publisher 集合里立即报错。
3. **跨节点参数一致性**：用 `ros2 param get` 拉 `/mcl_3dl`、`/pose_fusion`、`/lio_sam_mapOptimization` 的关键参数：
   - `mcl_3dl.publish_tf=false` 和 `publish_odom_tf=false`（localization 模式）
   - `num_particles_min ≤ num_particles ≤ num_particles_max`
   - `sub_map_search_radius` 在 0.3~1.5 倍 `lidar_max_range` 区间
   - `pose_fusion.publish_tf` 与 mode 对齐（localization=true、mapping=false）
   - `lio_sam.publishTF` 与 mode 对齐
   不一致全部走 ERROR diagnostic。

设计目标是 cheap：warmup_sec（默认 5s）后跑一次、recheck_sec（默认 30s）周期复查；节点没起来的检查项静默跳过，下个 tick 再试。绝不主动杀 launch，由人 / 上层根据诊断决定。

参数：

| Key | 默认 | 含义 |
|-----|------|------|
| `mode` | `"localization"` | `"mapping"` 或 `"localization"`，决定 TF / publish_tf 期望集 |
| `warmup_sec` | 5.0 | 第一次跑检查的延迟 |
| `recheck_sec` | 30.0 | 周期复查间隔 |
| `qos_topics` | 9 个高影响话题 | 要查 QoS 的话题列表 |
| `tf_unique_edges` | `["map->odom", "odom->base_link"]` | 期望唯一 owner 的边 |
| `lidar_max_range` | 70.0 m | 用来校验 `sub_map_search_radius` |

### Health / perf 阈值

* `slam_health_monitor`：默认 `OK` (≤ 0.5 s) → `WARN` (≤ 2 s) → `ERROR` (> 2 s) per topic / TF 边。建图模式下 launch 把 `ok_timeout` 放宽到 1.0 s、`fail_timeout` 到 3.0 s（LIO-SAM `mappingProcessInterval=0.1` + 单次优化偶发 100~150 ms 会让默认阈值产生周期性误报），同时把 `filtered_odom_topic` 置空（`pose_fusion` 不在建图链里）
* `nav_perf_monitor`：阈值跟着 100Hz odom + 5Hz planner 的预期速率走，慢于一半进 WARN，慢于五分之一进 ERROR。FAST-LIO 残差走自己的 `lio_residual_warn / error`

两个节点都只对 WARN/ERROR 出文字日志，OK 状态靠 `/diagnostics` 流式呈现。

### Bridge QoS 兼容性（坑点）

`livox_pc2_to_liosam` 的两个 publisher（`/livox/lidar_liosam` 和 `/livox/lidar_liosam_xyzi`）一律 **RELIABLE**，订阅侧 `/livox/lidar` 走 BEST_EFFORT 与 livox 驱动对齐。如果 publisher 改回 BEST_EFFORT，LIO-SAM `imageProjection` 因 `qos_lidar=RELIABLE` 不兼容直接收不到点云，整条后端管道（`cloud_deskewed`/`feature/cloud_info`/`mapping/odometry`/`map→odom` TF）全程空转，但表面上节点都活着，从外部看不出根因。改 QoS 时要保留 RELIABLE。

## 在系统中的角色

* 建图链：Livox driver → `livox_pc2_to_liosam` → FAST-LIO + LIO-SAM → `liosam_to_posegraph` 写 pose graph → `save_map_on_exit` 落盘
* 定位链：`dddnav_preflight` 做启动期 QoS / TF / 参数自检 → `std_global_init` 给初始位姿 + 在线 watchdog → MCL 收敛 → `slam_health_monitor` + `nav_perf_monitor` 在线监控

## STD 全局初始化 / watchdog 参数

通常默认就够，要改在 launch 里通过 `sc_init_yaml` 或者 CLI `-p` 传：

| Key | 默认 | 含义 |
|-----|------|------|
| `std_score_threshold` | 0.50 | 启动期 plane-ICP 最低 score（越大越严，0.3~0.7 常用） |
| `min_consensus_frames` | 3 | 启动期连续命中同一索引才确认 |
| `min_points` | 1500 | 输入点过少跳过查询 |
| `init_cov_xy / z / yaw / rp` | 1.0 / 0.25 / 0.09 / 0.04 | 发出去给 MCL 的协方差 |
| `enable_watchdog` | true | 关掉就退化为旧版"启动期一次性"行为 |
| `watchdog_check_hz` | 2.0 | watchdog 评估频率 |
| `watchdog_warmup_sec` | 8.0 | 首次发布后多久 watchdog 才上线 |
| `relocate_min_score` | 0.55 | watchdog 触发要求的最低 STD score（一般略高于启动期阈值） |
| `relocate_min_dist_from_live_m` | 4.0 | 候选关键帧距 MCL 当前关键帧的最低距离，避免 MCL 自洽时无意义重发 |
| `relocate_max_jump_m / _yaw` | 6.0 / 1.05 rad | 候选与 odom 先验的位置/朝向上限，杀掉跨地图错匹配 |
| `relocate_consensus` | 4 | 连续多少 watchdog tick 一致才再发 `/initial_3d_pose` |
| `relocate_holdoff_sec` | 10.0 | 发布后的冷却时间 |

无 `std_db.bin` 时节点直接退出，`runtime.yaml.initial_pose` 仍生效。
