# dddnav_pose_fusion

SE(3) error-state Kalman filter that fuses FAST-LIO 100 Hz odometry with MCL 3DL ~5 Hz global pose.

## What it does

- **Predict** with FAST-LIO `/Odometry` (body-frame relative deltas)
- **Update** with MCL 3DL `mcl_pose` in map frame
- **Output** `/odom_filtered` 100 Hz + `map → odom` TF

Loose coupling: FAST-LIO continues to publish `odom → base_link`; this node owns `map → odom`. MCL must run with `publish_tf=false` and `publish_odom_tf=false`.

## Files

| Path | Role |
|------|------|
| `include/dddnav_pose_fusion/eskf_se3.h` | ESKF math (right-perturbation, Joseph form) |
| `src/eskf_se3.cpp` | Rodrigues / log-SO(3) / predict / update |
| `src/pose_fusion_node.cpp` | ROS wiring |
| `config/pose_fusion.yaml` | Q / R covariance, gates, ZUPT, adaptive Q, auto-init |

## Tuning

Edit `config/pose_fusion.yaml`. The key knobs:

- `proc_noise_pos / rot` — bigger = trust MCL more, smaller = trust FAST-LIO more.
- `meas_noise_pos / rot` — measurement-noise floor; caps over-confident MCL covariance.
- `mahalanobis_gate` — chi²₆ threshold (16.81 = 99 %); reject MCL outliers.

### Robustness add-ons

- `zupt_lin_vel_thresh / zupt_ang_vel_thresh / zupt_proc_scale` — when both
  speeds are below the thresholds, predict uses `Q * zupt_proc_scale`.
  Stops the covariance from drifting while parked.
- `gate_reset_after` — after N consecutive Mahalanobis rejects, force-reset
  the filter at the latest MCL pose. Prevents staying stuck in the
  rejection regime after a sudden relocalisation.
- `max_cov_pos / max_cov_rot` — clamps the diagonal of P so a long MCL
  blackout cannot let the trace explode.
- `auto_init_timeout` — if no MCL pose arrives within this many seconds,
  the filter self-initialises at `(auto_init_x, _y, _z)`. Useful when MCL is
  late but downstream nodes need `/odom_filtered` to start.

### Adaptive Q (FAST-LIO residual-driven)

FAST-LIO publishes `/fast_lio/health` (`Float64MultiArray = [residual_m,
effective_feats]`). When residual rises above `adaptive_q_baseline`, the
predict step inflates Q by

    q_scale = 1 + adaptive_q_gain * (residual - baseline) / baseline    (capped at adaptive_q_max)

so MCL's global update wins more weight during jolts, dynamic crowds, or
featureless corridors. A separate `adaptive_q_min_feats` threshold also
boosts Q when correspondences fall off (front-end starving). Disable by
setting `adaptive_q_gain=0`.

## Run standalone

```bash
ros2 launch dddnav_pose_fusion pose_fusion.launch.py
```

Normally launched indirectly via `dddnav_bringup/localization*.launch.py`.
