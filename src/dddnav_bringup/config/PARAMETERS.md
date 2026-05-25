# 参数索引（dddnav_bringup）

整个导航栈的调参入口。**索引不复制参数值**，只指出每个参数在哪个文件里。改一处生效一处，避免两份不一致。

要找一个参数：分类 → 索引行 → 来源文件。

## 索引使用约定

* 每条参数标了"来源文件"。**只在来源文件改**。launch 通过 `parameters=[...]` 把这些 yaml 串接进来，加载顺序就是覆盖顺序。
* 参数名 / 含义 / 默认值与对应包 README 一致。本文档只做位置索引 + 调参建议。
* `tuning/` 是覆盖层目录。常见的现场调参在那里写小 overlay，不动默认 yaml。
* 跨节点联动的参数（如 ESKF `proc_noise_pos` ↔ MCL `update_min_d`）见"组合调参规则"段。

## 一图看清加载顺序

```
launch 入口 (mapping / mapping_nav / localization*.launch.py)
  └─► common_nodes.py 把以下文件按节点拼进 ros__parameters：

      runtime.yaml             启动级（外参 / 延时 / 驱动频率 / 初始位姿）
        ↓
      keyframes_mid360.yaml    建图模式：关键帧抽取阈值
        ↓
      LIO-SAM/config/params_mid360.yaml          建图模式：LIO-SAM 后端
      FAST_LIO/config/mid360_pc2.yaml            前端 IMU+LiDAR 噪声
        ↓
      dddnav_pose_fusion/config/pose_fusion.yaml ESKF / 退化状态机
        ↓
      nav/base.yaml + nav/<profile>.yaml          导航 / MCL / 感知
        ↓
      tuning/<override>.yaml                      （可选）覆盖层

后写入的覆盖前写入的（ROS 2 规范）。
```


## A. 启动 / 装机参数

启动级，与算法无关。决定机器人有什么、装在哪、什么时候启起来。

| 参数 | 含义 | 来源文件 |
|------|------|----------|
| `lidar_mount.{x,y,z,roll,pitch,yaw}` | base_link → livox_frame 外参（m / rad） | `runtime.yaml` |
| `camera_mount.{x,y,z,roll,pitch,yaw}` | base_link → camera_link 外参，仅 `*_with_camera` 用 | `runtime.yaml` |
| `livox_publish_freq` | Livox Mid360 发布频率（Hz）。改这里要同步 driver MID360_config | `runtime.yaml` |
| `delays.bridges/rviz/mcl_3dl/pose_fusion/...` | 各节点启动延时（s）。慢机加大 | `runtime.yaml` |
| `initial_pose.{x,y,z}` | 没有 SC db 时的兜底初始位姿 | `runtime.yaml` |

详见 `runtime.yaml` 顶部注释和 `dddnav_bringup/README.md` 的"目录"段。

## B. 雷达前端 / IMU 前端（FAST-LIO）

100 Hz 局部里程计 + IMU 预积分。改 IMU 噪声、点云预处理在这里。

| 参数 | 含义 | 来源文件 |
|------|------|----------|
| `preprocess.lidar_type` | `4 = MID360 (PointCloud2)`。换雷达型号必动 | `FAST_LIO/config/mid360_pc2.yaml` |
| `preprocess.scan_line / blind / scan_rate` | 扫描线数、盲区半径（m）、帧率（Hz） | 同上 |
| `mapping.acc_cov / gyr_cov` | IMU 噪声方差。**与 LIO-SAM imuAccNoise 同一物理量两份配置**，调参要同步 | 同上 |
| `mapping.b_acc_cov / b_gyr_cov` | IMU bias 随机游走 | 同上 |
| `mapping.det_range / fov_degree` | 雷达有效探测距离 / 视场角。换硬件必动 | 同上 |
| `mapping.extrinsic_T / extrinsic_R` | LiDAR↔IMU 外参，Mid360 同体单位阵 | 同上 |
| `feature_extract_enable / point_filter_num` | 是否抽角点 / 间隔降采样 | 同上 |
| `filter_size_surf / filter_size_map` | 体素大小，越小越准越慢 | 同上 |

调参建议见 `FAST_LIO/README*.md`。Mid360 IMU 噪声放大 10~50 倍是工程经验值，与 LIO-SAM `imuAccNoise=0.1` 同步。

## C. 雷达建图后端（LIO-SAM）

仅建图链路（mapping*.launch、mapping_nav*.launch）使用。定位模式不加载。

| 参数 | 含义 | 来源文件 |
|------|------|----------|
| `pointCloudTopic` | 接桥接后的点云，必须是 `/livox/lidar_liosam` | `LIO-SAM/config/params_mid360.yaml` |
| `imuAccNoise / imuGyrNoise / imuAccBiasN / imuGyrBiasN` | IMU 预积分噪声。与 FAST-LIO `mapping.acc_cov` 是同一组物理量 | 同上 |
| `imuRPYWeight` | 重力对齐权重。0.1~0.3 论文级，0.01 弱约束 | 同上 |
| `publishTF` | LIO-SAM 是否发 `map→odom`。**localization 模式必须 false** | 同上 |
| `numberOfCores / mappingProcessInterval` | 建图后端 CPU 利用 / 优化间隔（s） | 同上 |
| `surroundingkeyframeAddingDistThreshold/AngleThreshold` | LIO-SAM 关键帧抽取阈值 | 同上 |
| `loopClosureFrequency / historyKeyframeSearchRadius / historyKeyframeFitnessScore` | 回环检测频率 / 半径 / GICP 适合度阈值 | 同上 |
| `scExcludeRecent / scDistThreshold / scMinDatabase` | Scan Context 检索 | 同上 |

详细注释直接在 `params_mid360.yaml`，每段都标了严 / 宽方向。

## D. 关键帧 / pose graph 写盘

| 参数 | 含义 | 来源文件 |
|------|------|----------|
| `keyframe_dist / keyframe_angle` | 关键帧最小平移 / 旋转阈值 | `dddnav_bringup/config/keyframes_mid360.yaml` |
| `ground_angle_thresh` | 地面法线最大与垂直方向夹角（deg） | 同上 |
| `save_dir` | 输出目录。launch 自动覆盖到 `share/dddnav_bringup/map`，不用手动改 | 同上 |


## E. ESKF 融合 / 退化状态机（pose_fusion）

`/odom_filtered` 100 Hz、`map→odom` TF、定位质量与降级逻辑都在这里。

| 参数 | 含义 | 来源文件 |
|------|------|----------|
| 输入 / 输出话题、frame、`publish_tf` | ESKF 接线 | `dddnav_pose_fusion/config/pose_fusion.yaml` |
| `proc_noise_pos / proc_noise_rot` | 过程噪声，越大越信 MCL | 同上 |
| `meas_noise_pos / meas_noise_rot` | 量测噪声下限（MCL 协方差不可信时兜底） | 同上 |
| `mahalanobis_gate / adapt_gate_alpha / adapt_gate_max` | 拒绝离群 MCL，越紧越保守 | 同上 |
| `mcl_cov_reject_trace` | MCL 自报位置 cov trace 上限 (m²) | 同上 |
| `zupt_lin_vel_thresh / zupt_ang_vel_thresh / zupt_proc_scale` | 静止 ZUPT 缩 Q | 同上 |
| `lio_health_topic / adaptive_q_baseline / adaptive_q_gain / adaptive_q_max / adaptive_q_min_feats` | 自适应 Q（FAST-LIO 残差驱动） | 同上 |
| `gate_reset_after / max_cov_pos / max_cov_rot` | 连续拒绝重启 + 协方差上限 | 同上 |
| `auto_init_timeout / auto_init_x / _y / _z` | 无 MCL 兜底自启 | 同上 |
| `lio_blackout_sec / mcl_stuck_cov / mcl_stuck_sec / recovery_holdoff_sec` | 退化状态机阈值，HEALTHY ↔ DEGRADED_LIO_LOST ↔ DEGRADED_MCL_STUCK | 同上 |
| `status_topic / recovery_pub_topic / status_rate_hz / enable_recovery_publish` | `/localization_status` 输出与 SC 重定位重发 | 同上 |

含义沿用 `dddnav_pose_fusion/README.md`。

## F. 全局定位（MCL 3DL + sub_maps + SC 重定位）

定位链路核心。`base.yaml` 给所有 profile 共用的默认值，`nav/<profile>.yaml` 只写差异。

| 参数 | 含义 | 来源文件 |
|------|------|----------|
| `init_x / init_y / init_z / init_roll / init_pitch / init_yaw` 与 `init_var_*` | 启动时 MCL 粒子云均值 / 方差 | `nav/base.yaml`（mcl_3dl 段） |
| `num_particles / num_particles_min / num_particles_max / num_particles_grow_on_init` | 自适应粒子数上下界 + 重定位扩展 | 同上 |
| `match_ratio_grow_thresh / particle_decay` | 低 match → 翻倍粒子；健康 → 指数回落 | 同上 |
| `update_min_d / update_min_a` | 触发更新的最小行驶 / 转角 | 同上 |
| `odom_err_*` | 里程计噪声模型 | 同上 |
| `publish_tf / publish_odom_tf` | localization 启动时被强制改成 false（pose_fusion 拥有 TF） | 同上 + launch override |
| `expansion_var_*` | 全局扩展粒子的方差 | 同上 |
| `likelihood.*` | 似然模型阈值 | 同上 |
| `sub_map_search_radius / sub_map_warmup_trigger_distance` | 子图加载半径 + 预热触发距离 | 同上（sub_maps 段） |
| `complete_map_voxel_size` | 子图体素降采样 | 同上 |
| 启动时 SC 阈值 / watchdog | `sc_global_init` 节点参数（DB 路径自动注入） | `dddnav_bringup/launch/common_nodes.py::localization_stack` |

SC 节点完整参数表见 `dddnav_utils/README.md` 的"SC 全局初始化 / watchdog 参数"段。

## G. 局部规划 / 行为生成 / 评价器

| 参数 | 含义 | 来源文件 |
|------|------|----------|
| `controller_frequency / planner_patience / oscillation_*` | p2p_move_base 主循环 | `nav/base.yaml`（p2p_move_base 段） |
| `cuboid.{flb,frb,flt,frt,blb,brb,blt,brt}` | 机器人 8 角包围盒，碰撞用 | 同上（local_planner 段） |
| `differential_drive_simple.{max_vel_x, max_vel_theta, acc_lim_x, acc_lim_theta, sim_time, ...}` | 标准差速 trajectory generator | 同上（trajectory_generators 段） |
| `differential_drive_rotate_inplace / rotate_shortest_angle.rotation_speed / cuboid` | 原地旋转 / 最短朝向 | 同上 |
| `mpc_critics.{collision, stick_path, pure_pursuit, toward_global_plan, ...}.weight` | 评价器权重，按场景一组一起调 | 同上 |
| `recovery_behaviors.rotate_inplace.frequency / tolerance` | 卡住后的恢复行为 | 同上 |

机器人外形 / 速度 / 加速度强相关。换平台时只改 base.yaml 里 cuboid 与 differential_drive_simple 段即可。

## H. 全局规划

| 参数 | 含义 | 来源文件 |
|------|------|----------|
| `turning_weight / a_star_expanding_radius / enable_detail_log` | 全局规划评价 | `nav/base.yaml`（global_planner 段） |
| `use_pre_graph` | 静态地图缓存图（localization profile 打开） | `nav/<profile>.yaml`（global_planner 段） |
| `look_ahead_distance / recompute_frequency` | DWA 全局重规划 | `nav/base.yaml`（dynamic_window_aware_global_planner 段） |
| `global_planner_action_name / global_plan_query_frequency` | 模式相关 action 与重查频率，profile 必须重写 | `nav/<profile>.yaml`（global_plan_manager 段） |

## I. 感知（perception_3d）

`perception_3d_local` 给 local planner 喂 cost；`perception_3d_global` 给图规划喂可通行性。

| 参数 | 含义 | 来源文件 |
|------|------|----------|
| `inscribed_radius / inflation_radius / inflation_descending_rate` | 膨胀层 | `nav/<profile>.yaml`（perception_3d_local / _global 段） |
| `max_obstacle_distance / sensors_collected_frequency` | 障碍最大距离 / 采集频率 | 同上 |
| `plugins` | 启用的层（`map`, `lidar`, `path_blocked_strategy`, `depth_camera_layer`, ...） | 同上 |
| `lidar.vertical_FOV_top / _bottom / scan_effective_*` | 多层旋转雷达层的有效扇区 | 同上 |
| `lidar.resolution / xy_resolution / height_resolution / marking_height / perception_window_size` | 体素 / 标记高度 / 局部窗口 | 同上 |
| `map.is_local_planner / mapping_mode / map_topic / ground_topic` | 静态层加载源；mapping 模式 = true 启用动态地图层 | 同上 |
| 深度相机段（`depth_camera_layer`） | 仅 `*_with_camera`、`*_with_depth_camera` profile | `nav/mid360_*_with_camera.yaml` / `_with_depth_camera.yaml` |


## J. 健康监控 / 启动期自检

| 参数 | 含义 | 来源 |
|------|------|----------|
| `slam_health_monitor.{ok_timeout, fail_timeout, tf_jump_dist, tf_jump_angle, cov_trace_warn, cov_trace_error}` | SLAM 监控阈值 | `dddnav_bringup/launch/common_nodes.py::liosam_back_end`（建图模式覆盖） + 节点默认 |
| `nav_perf_monitor.{lio_residual_warn, lio_residual_error, lio_min_feats_warn}` | 导航性能阈值 | 节点参数（`scripts/nav_perf_monitor.py`） |
| `dddnav_preflight.{mode, warmup_sec, recheck_sec, qos_topics, tf_unique_edges, lidar_max_range}` | QoS / TF / 跨节点参数自检 | 节点参数（`scripts/dddnav_preflight.py`），launch 注入 `mode='localization'` |

完整说明见 `dddnav_utils/README.md`。

## 组合调参规则（牵一发动全身）

下面这些参数实际是一对，单调一个会失衡。

| 触发场景 | 一起调 |
|----------|--------|
| MCL 拖跟（FAST-LIO 跑得快但 ESKF 死信 LIO） | `pose_fusion.proc_noise_pos/_rot` ↑ + `mcl_3dl.update_min_d/_a` ↓ |
| MCL 跳得太狠（每次 update 后 `map→odom` 跳 > 0.3 m） | `pose_fusion.mahalanobis_gate` ↓ + `pose_fusion.adapt_gate_alpha` ↓ |
| 颠簸 / 长走廊里 LIO 残差 spike | `pose_fusion.adaptive_q_gain` ↑ + `pose_fusion.adaptive_q_max` ↑ |
| 重定位失败率高 | `mcl_3dl.num_particles_grow_on_init` ↑ + `mcl_3dl.match_ratio_grow_thresh` ↑ + SC `sc_dist_threshold` 收紧 |
| 静止时 cov trace 慢慢涨 | `pose_fusion.zupt_*` 三个一起 + `pose_fusion.max_cov_pos/_rot` 限制上界 |
| 子图加载抖动 | `sub_maps.sub_map_search_radius` 与 FAST-LIO `mapping.det_range` 比例 ≥ 0.5 |
| 雷达里程计稳定但 LIO-SAM 后端飘 | `LIO-SAM.imuAccNoise` 与 `FAST-LIO.mapping.acc_cov` 同步放大 |

## tuning/ 覆盖层

`config/tuning/` 提供小模板。复制一份改名后用 launch CLI 加载，**不要改默认 yaml**。

```bash
ros2 launch dddnav_bringup localization.launch.py \
  nav_profile:=mid360_localization \
  preflight_mode:=localization
```

需要再叠一层 overlay：

```bash
ros2 launch dddnav_bringup localization.launch.py \
  pose_fusion_yaml:=$(pwd)/src/dddnav_bringup/config/tuning/example_pose_fusion_overlay.yaml
```

`pose_fusion_yaml` / `nav_profile` 都已在 launch 里参数化，方便切。

## 我应该改哪里？速查

| 想改 | 去哪 |
|------|------|
| 雷达 / 相机的安装位置 | `runtime.yaml` |
| 启动顺序 / 延时 | `runtime.yaml.delays` |
| IMU / LiDAR 噪声 | `FAST_LIO/config/mid360_pc2.yaml` 与 `LIO-SAM/config/params_mid360.yaml` 同步 |
| ESKF / 退化阈值 | `dddnav_pose_fusion/config/pose_fusion.yaml` |
| MCL 粒子数 / 更新阈值 | `nav/base.yaml`（mcl_3dl 段） |
| 子图加载半径 | `nav/base.yaml`（sub_maps 段） |
| 机器人外形 / 速度 | `nav/base.yaml`（local_planner / trajectory_generators 段） |
| 评价器权重 | `nav/base.yaml`（mpc_critics 段） |
| 模式相关 (localization vs mapping) | `nav/<profile>.yaml` 单独覆盖 |
| 关键帧抽取 | `dddnav_bringup/config/keyframes_mid360.yaml` |
| 监控阈值 | `dddnav_utils/scripts/*.py` 节点参数 + launch 覆盖 |
