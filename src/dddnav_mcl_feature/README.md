# dddnav_mcl_feature

Feature extractor for MCL 3DL. Turns raw point clouds into the four edge/surface clouds that MCL's likelihood model expects.

## Pipeline

```
/livox/lidar_liosam_xyzi
        │
        ▼  imageProjection      → range image, segmented cloud, ground extraction
        │
        ▼  featureAssociation   → laser_cloud_{sharp, less_sharp, flat, less_flat}
        │
        ▼  MCL 3DL particle filter
```

## Outputs

- `laser_cloud_sharp` / `laser_cloud_less_sharp` — edge features
- `laser_cloud_flat`  / `laser_cloud_less_flat`  — surface features
- TF: usually does **not** publish `odom→base` (FAST-LIO owns it). Sanity-checks via tf2 lookup.

## Run

Launched by `dddnav_bringup/localization*.launch.py`. Configured via `mid360_localization*.yaml` in `p2p_move_base/config/`.

## Known issue (legacy)

`imageProjection` and `featureAssociation` overwrite the LiDAR sensor stamp with `clock_->now()` before publishing features. This was inherited from the upstream behaviour and MCL's approximate-time sync is tuned around it. Don't change without re-tuning MCL.
