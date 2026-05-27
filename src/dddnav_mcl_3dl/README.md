# dddnav_mcl_3dl

ROS 包名 **`mcl_3dl`**。基于 [at-wat/mcl_3dl](https://github.com/at-wat/mcl_3dl) 改的 3D 蒙特卡洛定位，适配地面机器人。

## 原理

* 粒子滤波，每个粒子是 6-DoF 位姿（x, y, z, roll, pitch, yaw）
* **预测**：用 FAST-LIO 输出的 `/Odometry` 做 odom 预积分
* **观测**：当前点云的 4 类特征（边 / 面 / 地面 / 强度）与子地图做最近邻匹配，按距离打分
* **重采样**：按行驶距离 / 转角触发，配合 KLD 风格的协方差扩展
* 不发 `map→odom`（`publish_tf=false`），交给 `dddnav_pose_fusion` 做高频融合后统一输出

## 子地图机制

地图按位姿图组织（`liosam_to_posegraph` 写到 `dddnav_bringup/map/`），运行时按机器人当前位置加载半径 `sub_map_search_radius` 的关键帧子图。子图尺度量级 ≈ `2 * lidar_detection_distance + 2 * sub_map_search_radius`。降算力，也方便多楼层。

## 在系统中的角色

定位流程的全局观测来源：

```
LiDAR ─► mcl_feature ─► mcl_3dl ─► /mcl_pose ─► pose_fusion (ESKF) ─► map→odom
                                              ▲
                              FAST-LIO /Odometry (100Hz 预测)
```

启动初始位姿现在由 `sc_global_init` 自动给（Scan Context 全局重定位），`runtime.yaml.initial_pose` 仍是兜底。

## 主要参数

集中在 `dddnav_bringup/config/nav_base.yaml` 的 `mcl_3dl` / `sub_maps` / `mcl_ip` / `mcl_fa` 段。常调：

| Key | 含义 |
|-----|------|
| `num_particles` | 粒子数稳态目标（默认 80，Mid360 点云质量好够用） |
| `num_particles_min` / `num_particles_max` | 自适应粒子数下/上界（KLD 风格的轻量替代） |
| `num_particles_grow_on_init` | 收到 `/initial_3d_pose` 时立即扩到的目标粒子数 |
| `match_ratio_grow_thresh` | match_ratio 低于该值翻倍粒子直到 max |
| `particle_decay` | 健康场景下指数回落系数（default 0.9，越小回落越快） |
| `update_min_d` / `update_min_a` | 触发更新的最小行驶/转角 |
| `likelihood.match_dist_min` / `match_dist_flat` | 边/面匹配距离阈值 |
| `sub_maps.sub_map_search_radius` | 子图搜索半径 |
| `publish_tf` / `publish_odom_tf` | 都关掉，TF 由 pose_fusion 与 FAST-LIO 接管 |

### 自适应粒子数

老逻辑只把 `num_particles` 当固定值，重定位时拿放宽 likelihood 凑数，搜索空间不够。现在：

1. 收到 `/initial_3d_pose` → 直接 `resizeParticle(num_particles_grow_on_init)`，给重定位足够的搜索预算
2. `measure()` 里 `match_ratio_max < match_ratio_grow_thresh` → 粒子数翻倍（capped at `num_particles_max`），同时重置 fix_cnt 让滤波器再静默几拍
3. 收敛后（fix_cnt 到 0、match_ratio 健康）每次 measure 按 `particle_decay` 指数回落，下界为 `max(num_particles, num_particles_min)`

`min >= max` 时整套机制关掉，行为退化到旧版。

## Bag 演示

`mcl_3dlXfeatureXbag.launch` 仍然可用（旧主线、Go2 / 上游样例）；Mid360 主线请用 `dddnav_bringup/localization*.launch.py`。
