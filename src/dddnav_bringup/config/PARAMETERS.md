# 参数调整指南

按**场景**找参数。每行的"在哪改"是真正修改 yaml 的位置。

参数名 / 含义 / 默认值与各包 README 一致，本文档只索引位置 + 联动建议。
**只在来源文件改**，launch 通过 `parameters=[...]` 串接，不要复制粘贴。

## 加载顺序

```
launch (mapping / mapping_nav / localization*.launch.py)
  └─► common_nodes.py 把以下文件按节点拼进 ros__parameters:

      reality/runtime.yaml                          启动级（外参 / 延时 / 初始位姿）
      reality/keyframes_mid360.yaml                 关键帧抽取阈值
      LIO-SAM/config/params_mid360.yaml             建图后端 (仅 mapping 链)
      FAST_LIO/config/mid360_pc2.yaml               IMU+LiDAR 前端
      reality/pose_fusion.yaml                      ESKF + 退化状态机
      nav_base.yaml + reality/nav/<profile>.yaml    导航 / MCL / 感知
      reality/tuning/<override>.yaml                可选覆盖层（CLI 注入）
```

后写入的覆盖前写入的。`reality/tuning/` 是覆盖层目录，**不动默认 yaml**。

---

## 场景速查

### 装机 / 标定

| 现象或目标 | 在哪改 |
|------------|--------|
| 雷达装机位置 | `reality/runtime.yaml` `lidar_mount.{x,y,z,roll,pitch,yaw}` |
| 相机装机位置（仅 `*_with_camera`） | `reality/runtime.yaml` `camera_mount.{x,y,z,roll,pitch,yaw}` |
| Mid360 发布频率 | `reality/runtime.yaml` `livox_publish_freq` + 同步驱动 `MID360_config.json` |
| 慢机启动顺序错乱 | `reality/runtime.yaml` `delays.{bridges, rviz, mcl_3dl, pose_fusion, ...}` 加大 |
| 没建图先跑定位的兜底起点 | `reality/runtime.yaml` `initial_pose.{x,y,z}`（有 `std_db.bin` 时被覆盖） |
| 换雷达型号 | `FAST_LIO/config/mid360_pc2.yaml` `preprocess.lidar_type / scan_line / scan_rate / blind` 一起改；`mapping.det_range / fov_degree / extrinsic_T / extrinsic_R` 同步 |

### 建图

| 现象或目标 | 在哪改 |
|------------|--------|
| 关键帧太密 / 太稀 | `reality/keyframes_mid360.yaml` `keyframe_dist`、`keyframe_angle` |
| 地面被错分类 | `reality/keyframes_mid360.yaml` `ground_angle_thresh`（deg） |
| 回环检测太慢 / 误闭环 | `LIO-SAM/config/params_mid360.yaml` `loopClosureFrequency / historyKeyframeSearchRadius / historyKeyframeFitnessScore` |
| STD 候选弱（save 后 `std_db.bin` 命中率低） | `LIO-SAM/config/params_mid360.yaml` `stdSkipNearNum / stdIcpThreshold / stdMinDatabase` |
| IMU 噪声不一致导致漂 | **同步两份**：`FAST_LIO/config/mid360_pc2.yaml` `mapping.{acc_cov, gyr_cov, b_acc_cov, b_gyr_cov}` ↔ `LIO-SAM/config/params_mid360.yaml` `imu{Acc,Gyr}{Noise,BiasN}` |
| 后端跑不动（CPU 满） | `LIO-SAM/config/params_mid360.yaml` `mappingProcessInterval` ↑ + `numberOfCores` ↑；同步把 `slam_health_monitor.ok_timeout` 放宽（建图链 launch 已设 1.0s） |
| 体素降太狠丢细节 / 太密太慢 | `FAST_LIO/config/mid360_pc2.yaml` `filter_size_surf` / `filter_size_map` |

### 全局重定位（STD）

| 现象或目标 | 在哪改 |
|------------|--------|
| 启动期定位失败 | `common_nodes.py::localization_stack` 注入的 `std_score_threshold` ↓（放宽）+ `min_consensus_frames` ↓ |
| 误匹配（长走廊 / 对称楼道） | `std_score_threshold` ↑ + `min_consensus_frames` ↑ + `relocate_min_dist_from_live_m` ↑ |
| 走错楼层 / 被搬运不恢复 | `enable_watchdog: true`（默认）+ `relocate_max_jump_m` 放宽到当前楼层尺度 |
| watchdog 过度触发 | `relocate_consensus` ↑ + `relocate_holdoff_sec` ↑ + `relocate_min_score` ↑ |

完整 STD 参数表：[`dddnav_utils/README.md` "STD 全局初始化 / watchdog 参数" 段](../../dddnav_utils/README.md)。

### MCL（粒子滤波收敛 / CPU 占用）

| 现象或目标 | 在哪改 |
|------------|--------|
| 重定位时粒子云太小、找不回 | `nav_base.yaml` `mcl_3dl.num_particles_grow_on_init` ↑ + `match_ratio_grow_thresh` ↑ |
| 收敛后粒子数没回落 | `nav_base.yaml` `mcl_3dl.particle_decay` ↓（更小回落更快） |
| MCL 拖跟（动得慢） | `nav_base.yaml` `mcl_3dl.update_min_d / update_min_a` ↓ |
| 子图加载抖动 | `nav_base.yaml` `sub_maps.sub_map_search_radius` 调到 ≥ 0.5 × `FAST-LIO.mapping.det_range` |
| likelihood 误匹配多 | `nav_base.yaml` `mcl_3dl.likelihood.match_dist_min / match_dist_flat` |
| MCL 想关掉自适应粒子数 | 把 `num_particles_min ≥ num_particles_max`（强制退化到旧版） |

快速试参用 [`reality/tuning/example_mcl_overlay.yaml`](reality/tuning/example_mcl_overlay.yaml)，`nav_profile:=` 或 CLI `--params-file` 叠上去。

### Pose fusion（ESKF 融合 / 退化）

来源文件统一是 [`reality/pose_fusion.yaml`](reality/pose_fusion.yaml)。

| 现象或目标 | 在哪改 |
|------------|--------|
| MCL 跳得太狠（每次 update 后 `map→odom` 跳 > 0.3 m） | `mahalanobis_gate` ↓ + `adapt_gate_alpha` ↓ |
| MCL 太信不上、跟不动 | `proc_noise_pos / proc_noise_rot` ↑（让 ESKF 更信 MCL） |
| 颠簸 / 长走廊 LIO 残差 spike | `adaptive_q_gain` ↑ + `adaptive_q_max` ↑（自适应 Q 借 FAST-LIO 残差驱动） |
| 静止时 cov trace 慢慢涨 | `zupt_lin_vel_thresh / zupt_ang_vel_thresh / zupt_proc_scale` 三件一起 + `max_cov_pos / max_cov_rot` 限上界 |
| MCL 自报 cov 太大想直接拒 | `mcl_cov_reject_trace` ↓（m²） |
| LIO 断流 1s+ 想触发 DEGRADED_LIO_LOST | `lio_blackout_sec` 改触发阈值；模式切换 WARN 打日志 |
| MCL 长时间 cov trace 高想触发 STUCK + 重发种子 | `mcl_stuck_cov` / `mcl_stuck_sec` / `recovery_holdoff_sec` 三件 |
| 想关掉退化时主动重发 `/initial_3d_pose` | `enable_recovery_publish: false`（仍发状态、不主动注入） |
| 启动一直没 MCL，想自启 | `auto_init_timeout` + `auto_init_x / _y / _z` |

试参覆盖：[`reality/tuning/example_pose_fusion_overlay.yaml`](reality/tuning/example_pose_fusion_overlay.yaml)，CLI:

```bash
ros2 launch dddnav_bringup localization.launch.py \
  pose_fusion_yaml:=$(pwd)/src/dddnav_bringup/config/reality/tuning/example_pose_fusion_overlay.yaml
```

### 局部规划 / 速度

`nav_base.yaml`，`local_planner` / `trajectory_generators` / `mpc_critics` 段。

| 现象或目标 | 在哪改 |
|------------|--------|
| 换底盘外形 | `cuboid.{flb, frb, flt, frt, blb, brb, blt, brt}` 一组（8 顶点 m）+ `inscribed_radius / inflation_radius` |
| 走廊里太慢 / 太快 | `differential_drive_simple.{max_vel_x, max_vel_theta, acc_lim_x, acc_lim_theta}` |
| 路径贴不上 / 走得抖 | `mpc_critics.stick_path.weight` ↑（更贴）；`mpc_critics.pure_pursuit.weight` ↑（更前瞻） |
| 起步朝向不对就硬走 | `mpc_critics.toward_global_plan.weight` ↑（先转再走，旋转 shim） |
| 转弯响应慢 | `differential_drive_simple.sim_time` ↓ 或 `sim_granularity` ↓（采样粒度变细） |
| 碰撞检查噪 | `mpc_critics.collision.weight` ↑；或 `differential_drive_simple.sim_granularity` ↓ |
| 卡住后想要不同 recovery | `recovery_behaviors.rotate_inplace.{frequency, tolerance}`；插槽留了，挂插件即可 |

### MPPI（可选采样规划器）

> 默认主线用 `differential_drive_simple`（DWA + cuboid + argmin）。MPPI 是叠加层，要主动启用。

启用 = 叠 [`reality/tuning/example_mppi_overlay.yaml`](reality/tuning/example_mppi_overlay.yaml) + 把 `p2p_move_base` 直行段的 `traj_gen_name` 切到 `differential_drive_mppi`（业务调度，overlay 不替你切）。

| 现象或目标 | 在哪改（都在 overlay 的 `differential_drive_mppi.mppi.*`） |
|------------|----------|
| 行为太激进想更平滑 | `lambda` ↑（softmax 温度大 → 多样本平均） |
| 行为太保守想更接近 argmin | `lambda` ↓ |
| 探索不够（贴住次优解） | `sigma_v / sigma_w` ↑（控制扰动方差），但会被 `mppi_stick_path` 罚分制约 |
| 嵌入式 CPU 不够 | `num_samples` 64 → 32 |
| 每周期想从 0 重启 nominal | `reset_nominal_each_cycle: true`（默认 false 用 warm start） |
| 大部分样本都被 critic 砍 | `min_valid_fraction` ↑；触发后 MPPI 自动降级 argmin |

完整说明：[`dddnav_local_planner/README.md`](../../dddnav_local_planner/README.md) MPPI 段。

### 全局规划

| 现象或目标 | 在哪改 |
|------------|--------|
| 路径绕远 / zig-zag | `nav_base.yaml` `global_planner.turning_weight` ↑ |
| 静态地图查询慢 | `reality/nav/<profile>.yaml` `global_planner.use_pre_graph: true`（预构图） |
| A\* 邻居展开太慢 / 太粗 | `nav_base.yaml` `global_planner.a_star_expanding_radius` |
| 边定位边规划，路径不刷新 | `reality/nav/<profile>.yaml` `global_plan_manager.global_plan_query_frequency` ↑（默认 5Hz） |
| DWA 全局重规划频率 | `nav_base.yaml` `dynamic_window_aware_global_planner.{look_ahead_distance, recompute_frequency}` |

### 感知 / 代价图（perception_3d）

profile-aware：默认值在 `nav_base.yaml`，模式覆盖在 `reality/nav/<profile>.yaml` 的 `perception_3d_local` / `perception_3d_global` 段。

| 现象或目标 | 在哪改 |
|------------|--------|
| 机器人离障碍太近 / 太远才反应 | `inscribed_radius / inflation_radius / inflation_descending_rate` |
| 远处障碍噪点（误标） | `max_obstacle_distance` ↓ |
| 想加 / 关图层 | `plugins:` 列表（`map / lidar / depth_camera_layer / path_blocked_strategy / ...`） |
| Mid360 扇区切片不准（继承自旋转雷达约定） | `lidar.{vertical_FOV_top, vertical_FOV_bottom, scan_effective_*, perception_window_size, segmentation_ignore_ratio}` |
| 体素 / 标记高度调整 | `lidar.{resolution, xy_resolution, height_resolution, marking_height}` |
| 静态层切到建图模式 | `map.is_local_planner / mapping_mode / map_topic / ground_topic` |
| 接 `*_with_camera` 语义点云 | `reality/nav/mid360_*_with_camera.yaml` 的 `depth_camera_layer` 段 |
| 限速区 / 禁入区 | `dddnav_perception_3d/config/{speed_limit_layer.yaml, no_entry_layer.yaml}` + 对应 PCD |

### 监控 / 自检

| 现象或目标 | 在哪改 |
|------------|--------|
| `slam_health_monitor` 误报 / 太迟报 | `common_nodes.py::liosam_back_end` 注入的 `ok_timeout / fail_timeout / tf_jump_dist / tf_jump_angle / cov_trace_warn / cov_trace_error` |
| FAST-LIO 残差告警阈值 | `dddnav_utils/scripts/nav_perf_monitor.py` 节点参数 `lio_residual_warn / lio_residual_error / lio_min_feats_warn` |
| Preflight 自检要换模式 / 加 topic | `dddnav_utils/scripts/dddnav_preflight.py` 节点参数 `mode / qos_topics / tf_unique_edges / lidar_max_range`；launch 默认注入 `mode='localization'` |

---

## 按文件分组速查

仅当场景表找不到时回退到这里。每段只列**这个文件里独有的**关键 key，不复制场景表。

### `reality/runtime.yaml`

`lidar_mount` / `camera_mount` / `livox_publish_freq` / `delays.*` / `initial_pose`。详见文件顶部注释。

### `reality/keyframes_mid360.yaml`

`keyframe_dist` / `keyframe_angle` / `ground_angle_thresh` / `save_dir`（launch 自动覆盖到 `share/dddnav_bringup/map/`，**不要手动改**）。

### `FAST_LIO/config/mid360_pc2.yaml`（前端）

| Key | 说明 |
|-----|------|
| `preprocess.{lidar_type, scan_line, blind, scan_rate}` | 雷达型号 / 扫描线 / 盲区 / 帧率 |
| `mapping.{acc_cov, gyr_cov, b_acc_cov, b_gyr_cov}` | IMU 噪声（Mid360 经验：放大 10~50 倍，与 LIO-SAM 同步） |
| `mapping.{det_range, fov_degree, extrinsic_T, extrinsic_R}` | 探测距离 / FOV / LiDAR↔IMU 外参 |
| `feature_extract_enable / point_filter_num` | 角点抽取 / 间隔降采样 |
| `filter_size_surf / filter_size_map` | 体素，越小越准越慢 |

调参建议：[`FAST_LIO/README.md`](../../FAST_LIO/README.md)。

### `LIO-SAM/config/params_mid360.yaml`（仅建图链）

| Key | 说明 |
|-----|------|
| `pointCloudTopic` | 必须是 `/livox/lidar_liosam` |
| `imu{Acc,Gyr}{Noise,BiasN}` | 与 FAST-LIO `mapping.acc_cov` 等同步 |
| `imuRPYWeight` | 重力对齐权重（论文 0.1~0.3，弱约束 0.01） |
| `publishTF` | localization 模式必须 `false`，由 launch 强制 |
| `numberOfCores / mappingProcessInterval` | 后端 CPU / 优化间隔（s） |
| `surroundingkeyframeAddingDistThreshold / AngleThreshold` | LIO-SAM 自身的关键帧抽取（与 `keyframes_mid360.yaml` 是不同链路） |
| `loopClosureFrequency / historyKeyframeSearchRadius / historyKeyframeFitnessScore` | 回环 |
| `stdSkipNearNum / stdIcpThreshold / stdMinDatabase` | STD 候选检索（替代上游的 Scan Context） |

`params_mid360.yaml` 每段都标了严 / 宽方向。

### `reality/pose_fusion.yaml`（ESKF + 退化）

四组 key：**Q/R/门** / **鲁棒性** / **自适应 Q** / **退化状态机**。完整含义见 [`dddnav_pose_fusion/README.md`](../../dddnav_pose_fusion/README.md)，场景速查见上方"Pose fusion"段。

### `nav_base.yaml`（导航公共层）

| 段 | 内容 |
|----|------|
| `mcl_3dl` | 初始位姿 / 自适应粒子数 / update 阈值 / odom 噪声 / likelihood / `publish_tf=false`（localization 强制） / `expansion_var_*` |
| `sub_maps` | `sub_map_search_radius / sub_map_warmup_trigger_distance / complete_map_voxel_size` |
| `p2p_move_base` | `controller_frequency / planner_patience / oscillation_*` |
| `local_planner` | `cuboid` / 各 trajectory generator 速度 + sim_time + sim_granularity |
| `trajectory_generators` | `differential_drive_simple / differential_drive_rotate_inplace / differential_drive_rotate_shortest_angle`，启用 MPPI 时叠 overlay |
| `mpc_critics` | `collision / stick_path / pure_pursuit / toward_global_plan / ...` weight |
| `recovery_behaviors` | `rotate_inplace.{frequency, tolerance}` |
| `global_planner` | `turning_weight / a_star_expanding_radius / enable_detail_log` |
| `dynamic_window_aware_global_planner` | `look_ahead_distance / recompute_frequency` |
| `perception_3d_local` / `perception_3d_global` | 默认膨胀 / FOV / 体素，profile 用 overlay 改 |

### `reality/nav/<profile>.yaml`（模式 overlay）

只写差异。常见：`global_plan_manager.{global_planner_action_name, global_plan_query_frequency}`、`global_planner.use_pre_graph`、`perception_3d_*.plugins / depth_camera_layer / map.{is_local_planner, mapping_mode, map_topic, ground_topic}`。

可用 profile：`mid360_mapping[_with_camera] / mid360_localization[_with_camera/_with_depth_camera]`。

---

## 联动规则（牵一发动全身）

单调一个会失衡，必须配对调。

| 场景 | 一起调 |
|------|--------|
| MCL 拖跟（FAST-LIO 跑得快但 ESKF 死信 LIO） | `pose_fusion.proc_noise_pos/_rot` ↑ + `mcl_3dl.update_min_d/_a` ↓ |
| MCL 跳得太狠 | `pose_fusion.mahalanobis_gate` ↓ + `pose_fusion.adapt_gate_alpha` ↓ |
| 颠簸 / 长走廊 LIO 残差 spike | `pose_fusion.adaptive_q_gain` ↑ + `pose_fusion.adaptive_q_max` ↑ |
| 重定位失败率高 | `mcl_3dl.num_particles_grow_on_init` ↑ + `mcl_3dl.match_ratio_grow_thresh` ↑ + STD `std_score_threshold` 收紧（↑） |
| 静止时 cov trace 慢慢涨 | `pose_fusion.zupt_*` 三件 + `pose_fusion.{max_cov_pos, max_cov_rot}` 限上界 |
| 子图加载抖动 | `sub_maps.sub_map_search_radius` ≥ 0.5 × `FAST-LIO.mapping.det_range` |
| 雷达里程计稳但 LIO-SAM 后端飘 | `LIO-SAM.imu{Acc,Gyr}Noise` 与 `FAST-LIO.mapping.{acc_cov, gyr_cov}` 同步放大 |
| MPPI 启用后路径不贴 | `mppi.lambda` ↑（更平滑）+ `mppi_stick_path.weight` ↑ |
| MPPI 探索不够 | `mppi.sigma_v / sigma_w` ↑ + `mppi.num_samples` ↑（CPU 允许） |
| 后端 CPU 满 + health 误报 | `LIO-SAM.mappingProcessInterval` ↑ + `slam_health_monitor.{ok_timeout, fail_timeout}` 同步放宽 |

---

## 用 tuning/ 覆盖层试参

`reality/tuning/` 提供小模板。**复制改名**后用 launch CLI 加载，**不要改默认 yaml**：

```bash
# 切 nav profile
ros2 launch dddnav_bringup localization.launch.py nav_profile:=mid360_localization_with_depth_camera

# 叠加 pose_fusion overlay
ros2 launch dddnav_bringup localization.launch.py \
  pose_fusion_yaml:=$(pwd)/src/dddnav_bringup/config/reality/tuning/example_pose_fusion_overlay.yaml

# MPPI（叠 overlay + 切直行 generator 名）
ros2 launch dddnav_bringup localization.launch.py \
  p2p_move_base_yaml:=$(pwd)/src/dddnav_bringup/config/reality/tuning/example_mppi_overlay.yaml
```

`pose_fusion_yaml` / `nav_profile` / `p2p_move_base_yaml` 已在 launch 里参数化。新做 profile 直接复制 `reality/nav/mid360_localization.yaml` 改差异部分即可。
