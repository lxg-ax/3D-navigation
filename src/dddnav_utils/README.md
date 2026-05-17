# dddnav_utils

Glue / adapter nodes shared across the stack.

## Nodes

| Executable | Lang | Role |
|------------|------|------|
| `livox_pc2_to_liosam` | C++ | Livox PointCloud2 → VelodynePointXYZIRT (LIO-SAM) + PointXYZI (MCL). On the 10Hz data path; C++ to avoid GIL jitter |
| `livox_pc2_to_liosam.py` | Python | Reference / fallback of the above |
| `liosam_to_posegraph.py` | Python | Subscribes LIO-SAM keyframes, writes binary PCDs + pose graph to `dddnav_bringup/map/` |
| `fastlio_to_posegraph.py` | Python | Same as above but for pure FAST-LIO sessions |
| `slam_health_monitor.py` | Python | Watchdog: checks `/Odometry`, `lio_sam/mapping/odometry` topic liveness + key TF edges (`map→odom`, `odom→base_link`) |

## Save pose graph

```bash
ros2 service call /save_liosam_posegraph std_srvs/srv/Empty {}
```

## Health monitor states

`OK` (≤ 0.5 s) → `WARN` (≤ 2 s) → `FAIL` (> 2 s). Logs only, doesn't restart anything.
