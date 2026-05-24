# dddnav_utils

Glue / adapter nodes shared across the stack.

## Nodes

| Executable | Lang | Role |
|------------|------|------|
| `livox_pc2_to_liosam` | C++ | Livox PointCloud2 → VelodynePointXYZIRT (LIO-SAM) + PointXYZI (MCL). On the 10 Hz data path; C++ to avoid GIL jitter. |
| `sc_global_init` | C++ | Scan Context global localiser. Loads `share/dddnav_bringup/map/lio_sam/sc_db.bin` + `poses.pcd`, queries the live LiDAR scan, publishes `/initial_3d_pose` once a confident match is found. Replaces the operator-supplied initial pose for kidnapped-robot bootstrap. |
| `liosam_to_posegraph.py` | Python | Subscribes LIO-SAM keyframes, writes binary PCDs + pose graph to `dddnav_bringup/map/`. |
| `fastlio_to_posegraph.py` | Python | Same as above for pure FAST-LIO sessions. |
| `slam_health_monitor.py` | Python | Watchdog: topic liveness, TF edge liveness, `map→odom` jump detector, `/odom_filtered` covariance trace. Publishes `diagnostic_msgs/DiagnosticArray` on `/diagnostics`. |
| `nav_perf_monitor.py` | Python | Runtime telemetry: rate / age on `/Odometry`, `/odom_filtered`, `cmd_vel`, `/global_planner/path`, plus FAST-LIO residual / feature count. Also publishes to `/diagnostics`. |
| `save_map_on_exit.py` | Python | Atomic `pose_graph → LIO-SAM dump` save sequence. Used by `mapping*.launch.py` when `auto_save_on_exit:=true`. |

## Save pose graph

Manual two-step (legacy):

```bash
ros2 service call /save_liosam_posegraph std_srvs/srv/Empty {}
ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.2}"
```

Or let the launch fire `save_map_on_exit.py` for you:

```bash
ros2 launch dddnav_bringup mapping.launch.py auto_save_on_exit:=true
# Ctrl-C 后, 自动按顺序: pose graph -> /lio_sam/save_map
```

LIO-SAM 在 save 时会同时把 Scan Context 数据库写到 `<dest>/sc_db.bin` (body 帧描述符)，定位时被 `sc_global_init` 直接消费。

## SC 全局初始化

`localization*.launch.py` 默认启动 `sc_global_init`：

* 加载 `share/dddnav_bringup/map/lio_sam/sc_db.bin` 与 `share/dddnav_bringup/map/poses.pcd`
* 订阅 `/livox/lidar_liosam_xyzi` (PointXYZI, base 系)
* 每帧重建 SC 描述符并查表，连续 `min_consensus_frames` 次匹配同一关键帧 → 发 `/initial_3d_pose` 后退出搜索
* 协方差初始 ~1m / ~0.3 rad，留给 MCL 微调

参数（在 `localization_stack(..., sc_init_yaml=...)` 里覆盖，或在 launch CLI 上 `-p`）：

| Key | Default | Notes |
|-----|---------|-------|
| `sc_dist_threshold` | 0.30 | 余弦距离阈值，越小越严格 |
| `min_consensus_frames` | 3 | 连续命中同一索引才确认 |
| `min_points` | 1500 | 输入点过少跳过查询 |
| `init_cov_xy / z / yaw / rp` | 1.0 / 0.25 / 0.09 / 0.04 | 发出去给 MCL 的协方差 |

无 `sc_db.bin` 时节点直接退出，`runtime.yaml.initial_pose` 仍然兜底。

## Health / perf monitor states

`slam_health_monitor.py`：`OK` (≤ 0.5 s) → `WARN` (≤ 2 s) → `ERROR` (> 2 s) per topic / TF edge。Diagnostic 面板看颜色就行。

`nav_perf_monitor.py`：阈值跟着 100 Hz odom + 5 Hz planner 的预期速率走，慢于一半进 WARN，慢于五分之一进 ERROR；FAST-LIO 残差走自己的 `lio_residual_warn / error` 阈值。两个节点 logs 只对 WARN/ERROR 出文字，其余靠 `/diagnostics` 流式。
