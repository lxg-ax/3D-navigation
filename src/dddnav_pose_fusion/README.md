# dddnav_pose_fusion

SE(3) error-state Kalman filter that fuses FAST-LIO 100Hz odometry with MCL 3DL ~5Hz global pose.

## What it does

- **Predict** with FAST-LIO `/Odometry` (body-frame relative deltas)
- **Update** with MCL 3DL `mcl_pose` in map frame
- **Output** `/odom_filtered` 100Hz + `map → odom` TF

Loose coupling: FAST-LIO continues to publish `odom → base_link`; this node owns `map → odom`. MCL must run with `publish_tf=false` and `publish_odom_tf=false`.

## Files

| Path | Role |
|------|------|
| `include/dddnav_pose_fusion/eskf_se3.h` | ESKF math (right-perturbation, Joseph form) |
| `src/eskf_se3.cpp` | Rodrigues / log-SO(3) / predict / update |
| `src/pose_fusion_node.cpp` | ROS wiring |
| `config/pose_fusion.yaml` | Q / R covariance, Mahalanobis gate |

## Tuning

Edit `config/pose_fusion.yaml`:

- `proc_noise_pos / rot` — bigger = trust MCL more, smaller = trust FAST-LIO more
- `meas_noise_pos / rot` — measurement-noise floor (caps overconfident MCL covariance)
- `mahalanobis_gate` — chi²₆ threshold (16.81 = 99%); reject MCL outliers

## Run standalone

```bash
ros2 launch dddnav_pose_fusion pose_fusion.launch.py
```

Normally launched indirectly via `dddnav_bringup/localization*.launch.py`.
