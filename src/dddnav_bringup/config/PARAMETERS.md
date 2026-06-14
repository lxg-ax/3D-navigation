# 参数调整指南

按场景找参数。每行的"在哪改"是真正修改 yaml 的位置；只在来源文件改，launch 自动串接。

## 加载顺序

```
launch (mapping / mapping_nav / localization*.launch.py)
  └─► common_nodes.py 把以下文件按节点拼进 ros__parameters:

      reality/runtime.yaml                          启动级（外参 / 延时 / 初始位姿）
      reality/keyframes_mid360.yaml                 关键帧抽取阈值
      reality/slam/liosam_mid360.yaml               LIO-SAM 后端 (仅 mapping 链)
      reality/slam/fastlio_mid360.yaml              FAST-LIO 前端
      reality/pose_fusion.yaml                      ESKF + 退化状态机
      nav_base.yaml + reality/nav/<profile>.yaml    导航 / MCL / 感知
      reality/tuning/<override>.yaml                可选覆盖层（CLI 注入）
```

后写覆盖前写。`reality/tuning/` 是覆盖层目录，不动默认 yaml。

> SLAM 调参（FAST-LIO/LIO-SAM）已搬到 `reality/slam/` 集中管理，原 `src/FAST_LIO/config/` 与 `src/LIO-SAM/config/` 不生效。

---

## 场景速查

### 装机 / 标定

| 现象或目标 | 在哪改 |
|------------|--------|
| 雷达装机位置 | `reality/runtime.yaml` `lidar_mount.{x,y,z,roll,pitch,yaw}` |
| 相机装机位置（仅 `*_with_camera`） | `reality/runtime.yaml` `camera_mount.*` |
| Mid360 发布频率 | `reality/runtime.yaml` `livox_publish_freq` + 同步驱动 `MID360_config.json` |
| 慢机启动顺序错乱 | `reality/runtime.yaml` `delays.*` 加大 |
| 没建图先跑定位的兜底起点 | `reality/runtime.yaml` `initial_pose.*`（有 `std_db.bin` 时被覆盖） |
| 换雷达型号 | `reality/slam/fastlio_mid360.yaml` `preprocess.{lidar_type, scan_line, scan_rate, blind}` + `mapping.{det_range, fov_degree, extrinsic_T, extrinsic_R}` |

### 建图

| 现象或目标 | 在哪改 |
|------------|--------|
| 关键帧太密 / 太稀 | `reality/keyframes_mid360.yaml` `keyframe_dist`、`keyframe_angle` |
| 地面被错分类 | `reality/keyframes_mid360.yaml` `ground_angle_thresh`（deg） |
| 回环检测太慢 / 误闭环 | `reality/slam/liosam_mid360.yaml` `loopClosureFrequency / historyKeyframeSearchRadius / historyKeyframeFitnessScore` |
| STD 候选弱 | `reality/slam/liosam_mid360.yaml` `stdSkipNearNum / stdIcpThreshold / stdMinDatabase` |
| IMU 噪声漂移 | FAST-LIO `mapping.{acc_cov,gyr_cov,b_*_cov}` ↔ LIO-SAM `imu{Acc,Gyr}{Noise,BiasN}`，**语义不同见 [`IMU_NOTES.md`](reality/slam/IMU_NOTES.md)** |
| 后端跑不动（CPU 满） | `reality/slam/liosam_mid360.yaml` `mappingProcessInterval` ↑ + `numberOfCores` ↑ |
| 体素降太狠丢细节 / 太密太慢 | `reality/slam/fastlio_mid360.yaml` `filter_size_surf` / `filter_size_map` |

### 全局重定位（STD）

| 现象或目标 | 在哪改 |
|------------|--------|
| 启动期定位失败 | `common_nodes.py::localization_stack` 注入的 `std_score_threshold` ↓ + `min_consensus_frames` ↓ |
| 误匹配（长走廊 / 对称楼道） | `std_score_threshold` ↑ + `min_consensus_frames` ↑ + `relocate_min_dist_from_live_m` ↑ |
| 走错楼层 / 被搬运不恢复 | `enable_watchdog: true`（默认）+ `relocate_max_jump_m` 放宽到楼层尺度 |
| watchdog 过度触发 | `relocate_consensus` ↑ + `relocate_holdoff_sec` ↑ + `relocate_min_score` ↑ |

完整 STD 参数：[`dddnav_utils/README.md`](../../dddnav_utils/README.md)。

### MCL（粒子滤波）

| 现象或目标 | 在哪改 |
|------------|--------|
| 重定位时粒子云太小 | `nav_base.yaml` `mcl_3dl.num_particles_grow_on_init` ↑ + `match_ratio_grow_thresh` ↑ |
| 收敛后粒子数没回落 | `nav_base.yaml` `mcl_3dl.particle_decay` ↓ |
| MCL 拖跟（动得慢） | `nav_base.yaml` `mcl_3dl.update_min_d / update_min_a` ↓ |
| 子图加载抖动 | `nav_base.yaml` `sub_maps.sub_map_search_radius` ≥ 0.5 × `FAST-LIO.mapping.det_range` |
| likelihood 误匹配多 | `nav_base.yaml` `mcl_3dl.likelihood.match_dist_min / match_dist_flat` |
| 想关掉自适应粒子数 | `num_particles_min ≥ num_particles_max` |

试参用 [`tuning/example_mcl_overlay.yaml`](reality/tuning/example_mcl_overlay.yaml)。

### Pose fusion（ESKF / 退化）

来源文件：[`reality/pose_fusion.yaml`](reality/pose_fusion.yaml)。

| 现象或目标 | 在哪改 |
|------------|--------|
| MCL 跳得太狠（`map→odom` 跳 > 0.3 m） | `mahalanobis_gate` ↓ + `adapt_gate_alpha` ↓ |
| MCL 太信不上、跟不动 | `proc_noise_pos / proc_noise_rot` ↑ |
| 颠簸 / 长走廊 LIO 残差 spike | `adaptive_q_gain` ↑ + `adaptive_q_max` ↑ |
| 静止时 cov trace 慢慢涨 | `zupt_*` 三件 + `max_cov_pos / max_cov_rot` 限上界 |
| MCL 自报 cov 太大想拒收 | `mcl_cov_reject_trace` ↓（m²） |
| LIO 断流 1s+ 触发降级 | `lio_blackout_sec` |
| MCL 卡死想触发 STUCK + 重发种子 | `mcl_stuck_cov` / `mcl_stuck_sec` / `recovery_holdoff_sec` |
| 关掉退化时主动重发 `/initial_3d_pose` | `enable_recovery_publish: false` |
| 启动一直没 MCL，自启 | `auto_init_timeout` + `auto_init_x/_y/_z` |

试参用 [`tuning/example_pose_fusion_overlay.yaml`](reality/tuning/example_pose_fusion_overlay.yaml)。

### 局部规划

`nav_base.yaml`，`local_planner` / `trajectory_generators` / `mpc_critics` 段。

| 现象或目标 | 在哪改 |
|------------|--------|
| 换底盘外形 | `cuboid.{flb,frb,flt,frt,blb,brb,blt,brt}` 8 顶点 + `inscribed_radius / inflation_radius` |
| 走廊里太慢 / 太快 | `differential_drive_simple.{max_vel_x, max_vel_theta, acc_lim_*}` |
| 路径贴不上 / 走得抖 | `mpc_critics.stick_path.weight` ↑；`pure_pursuit.weight` ↑ |
| 起步朝向不对硬走 | `mpc_critics.toward_global_plan.weight` ↑ |
| 转弯响应慢 | `differential_drive_simple.sim_time` ↓ 或 `sim_granularity` ↓ |

### MPPI（可选采样规划器）

> 默认主线 `differential_drive_simple`。MPPI 是叠加层，要主动启用：叠 [`tuning/example_mppi_overlay.yaml`](reality/tuning/example_mppi_overlay.yaml) + 在 `p2p_move_base` 把直行段 `traj_gen_name` 切到 `differential_drive_mppi`。

| 现象或目标 | 在哪改（overlay 的 `differential_drive_mppi.mppi.*`） |
|------------|----------|
| 行为太激进想更平滑 | `lambda` ↑ |
| 探索不够（贴次优解） | `sigma_v / sigma_w` ↑ |
| 嵌入式 CPU 不够 | `num_samples` 64 → 32 |
| 大部分样本被 critic 砍 | `min_valid_fraction` ↑（自动降级到 argmin） |

完整说明：[`dddnav_local_planner/README.md`](../../dddnav_local_planner/README.md)。

### 全局规划

| 现象或目标 | 在哪改 |
|------------|--------|
| 路径绕远 / zig-zag | `nav_base.yaml` `global_planner.turning_weight` ↑ |
| 静态地图查询慢 | `reality/nav/<profile>.yaml` `global_planner.use_pre_graph: true` |
| A\* 邻居展开太慢 / 太粗 | `nav_base.yaml` `global_planner.a_star_expanding_radius` |
| 边定位边规划路径不刷新 | `reality/nav/<profile>.yaml` `global_plan_manager.global_plan_query_frequency` ↑ |
| DWA 全局重规划频率 | `nav_base.yaml` `dynamic_window_aware_global_planner.{look_ahead_distance, recompute_frequency}` |

### 感知 / 代价图

profile-aware：默认在 `nav_base.yaml`，模式 overlay 在 `reality/nav/<profile>.yaml` `perception_3d_local / perception_3d_global` 段。

| 现象或目标 | 在哪改 |
|------------|--------|
| 离障碍太近/太远才反应 | `inscribed_radius / inflation_radius / inflation_descending_rate` |
| 远处障碍噪点 | `max_obstacle_distance` ↓ |
| 加 / 关图层 | `plugins:` 列表 |
| Mid360 扇区切片不准 | `lidar.{vertical_FOV_*, scan_effective_*, perception_window_size, segmentation_ignore_ratio}` |
| 体素 / 标记高度 | `lidar.{resolution, xy_resolution, height_resolution, marking_height}` |
| 静态层切到建图模式 | `map.{is_local_planner, mapping_mode, map_topic, ground_topic}` |
| 接 `*_with_camera` 语义点云 | `reality/nav/mid360_*_with_camera.yaml` `depth_camera_layer` 段 |
| 限速区 / 禁入区 | `dddnav_perception_3d/config/{speed_limit_layer, no_entry_layer}.yaml` + 对应 PCD |

### 监控 / 自检

| 现象或目标 | 在哪改 |
|------------|--------|
| `slam_health_monitor` 误报 / 太迟 | `common_nodes.py::liosam_back_end` 注入的 `ok_timeout / fail_timeout / tf_jump_* / cov_trace_*` |
| FAST-LIO 残差告警 | `nav_perf_monitor.py` 节点参数 `lio_residual_warn / lio_residual_error / lio_min_feats_warn` |
| Preflight 换模式 / 加 topic | `dddnav_preflight.py` 节点参数 `mode / qos_topics / tf_unique_edges / lidar_max_range` |

---

## 联动规则（牵一发动全身）

单调一个会失衡，必须配对调。

| 场景 | 一起调 |
|------|--------|
| MCL 拖跟（FAST-LIO 跑得快但 ESKF 死信 LIO） | `pose_fusion.proc_noise_pos/_rot` ↑ + `mcl_3dl.update_min_d/_a` ↓ |
| MCL 跳得太狠 | `pose_fusion.mahalanobis_gate` ↓ + `adapt_gate_alpha` ↓ |
| 颠簸 / 长走廊 LIO 残差 spike | `pose_fusion.adaptive_q_gain` ↑ + `adaptive_q_max` ↑ |
| 重定位失败率高 | `mcl_3dl.num_particles_grow_on_init` ↑ + `match_ratio_grow_thresh` ↑ + STD `std_score_threshold` ↑ |
| 静止时 cov trace 慢慢涨 | `pose_fusion.zupt_*` 三件 + `max_cov_pos/_rot` 限上界 |
| 子图加载抖动 | `sub_maps.sub_map_search_radius` ≥ 0.5 × `FAST-LIO.mapping.det_range` |
| LIO-SAM 后端飘 | `LIO-SAM.imu{Acc,Gyr}Noise` 与 `FAST-LIO.mapping.{acc_cov, gyr_cov}` 同步放大（[`IMU_NOTES.md`](reality/slam/IMU_NOTES.md)） |
| MPPI 启用后路径不贴 | `mppi.lambda` ↑ + `mppi_stick_path.weight` ↑ |
| 后端 CPU 满 + health 误报 | `LIO-SAM.mappingProcessInterval` ↑ + `slam_health_monitor.{ok_timeout, fail_timeout}` 同步放宽 |

---

## 用 tuning/ 覆盖层试参

`reality/tuning/` 提供 overlay 模板。复制改名后用 launch CLI 加载，不要改默认 yaml。
完整说明见 [`tuning/README.md`](reality/tuning/README.md)。
