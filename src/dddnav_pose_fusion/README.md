# dddnav_pose_fusion

SE(3) 上的误差状态卡尔曼（ESKF），融合 FAST-LIO 100Hz 局部里程计与 MCL 3DL ~5Hz 全局位姿，输出 100Hz `/odom_filtered` 并发布 `map → odom`。

## 原理

* **状态**：在 map 系下机器人的 6-DoF 位姿（nominal: position + quaternion；error: tangent 6 维）
* **预测**：用 FAST-LIO 两次 odometry 的 body-frame 相对位姿 `T_b1_b2` 推一步 nominal，按 `dt` 累积过程噪声 Q
* **量测**：MCL 输出的 map-frame 位姿 `(p, q)` 当作 6 维量测，残差在 tangent 上（位置差 + `log(q⁻¹ q_meas)`）
* **更新**：Joseph form 做协方差更新，`χ²₆` Mahalanobis 门拒绝离群
* **TF 输出**：`map → odom = T_m_b * T_o_b⁻¹`，配合 FAST-LIO 自发的 `odom → base_link`

为什么不用普通 EKF：四元数有单位约束，直接在 4 维上做协方差更新会破坏正交性。误差状态在 tangent（旋转 3 维 + 位置 3 维）上是无约束的，最干净。

## 在系统中的角色

定位流程的"低延迟黏合层"：

```
FAST-LIO  /Odometry (100Hz)  ─► predict ─► /odom_filtered (100Hz)
                                          map → odom (TF)
MCL 3DL   /mcl_pose  (~5Hz)  ─► update
```

MCL 必须 `publish_tf=false` + `publish_odom_tf=false`，TF 单一所有者由本节点持有。

## 主要参数

`config/pose_fusion.yaml`，分四组：

**Q / R / 门**

| Key | 含义 |
|-----|------|
| `proc_noise_pos` / `proc_noise_rot` | 过程噪声标准差，越大越信 MCL |
| `meas_noise_pos` / `meas_noise_rot` | 量测噪声下限（MCL 协方差不可信时兜底） |
| `mahalanobis_gate` | χ²₆ 阈值基础值，默认 16.81（99%） |
| `adapt_gate_alpha` | 自适应门系数：`gate = mahalanobis_gate + α·trace(P_mcl_xyz)` |
| `adapt_gate_max` | 自适应门上限，避免门永远开着失去拒收能力 |
| `mcl_cov_reject_trace` | MCL 自身位置协方差 trace 超阈直接丢量测（m²），杀掉未收敛的 MCL |

**鲁棒性增强**

| Key | 含义 |
|-----|------|
| `zupt_lin_vel_thresh` / `zupt_ang_vel_thresh` / `zupt_proc_scale` | 静止时 Q 乘 scale，避免协方差空转上涨 |
| `gate_reset_after` | 连续 N 次门拒绝后强制重启 filter，防卡死在拒绝态 |
| `max_cov_pos` / `max_cov_rot` | P 对角线上限，避免 MCL 长时间没消息时 trace 爆炸 |
| `auto_init_timeout` | 超时无 MCL 时用 `auto_init_xyz` 自启 |

**自适应 Q（FAST-LIO 残差驱动）**

订阅 `/fast_lio/health = [scan_to_map_residual_m, effective_feats]`，公式：

```
q_scale = 1 + adaptive_q_gain * max(0, residual - baseline) / baseline   （capped at adaptive_q_max）
```

颠簸 / 动态 / 长走廊时让 MCL 拿更大权重；feats 不足同样会 boost 到 ≥4x。`adaptive_q_gain=0` 关掉。

## 文件

| 路径 | 内容 |
|------|------|
| `include/dddnav_pose_fusion/eskf_se3.h` | ESKF 数学（右扰动、Joseph form） |
| `src/eskf_se3.cpp` | Rodrigues / log-SO(3) / predict / update |
| `src/pose_fusion_node.cpp` | ROS 接线 |
| `config/pose_fusion.yaml` | 全部调参 |

## 单独启动

```bash
ros2 launch dddnav_pose_fusion pose_fusion.launch.py
```

通常在 `dddnav_bringup/localization*.launch.py` 里间接拉起。
