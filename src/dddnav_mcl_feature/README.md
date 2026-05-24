# dddnav_mcl_feature

MCL 3DL 的特征提取器：原始点云 → 4 类特征（边 / 强边 / 面 / 强面）+ 地面分割。

## 数据流

```
/livox/lidar_liosam_xyzi
        │
        ▼  imageProjection      把点投到 range image，做点云分割与地面提取
        │
        ▼  featureAssociation   提 sharp / less_sharp / flat / less_flat 特征
        │
        ▼  mcl_3dl 粒子滤波
```

## 输出话题

| 话题 | 类型 | 含义 |
|------|------|------|
| `laser_cloud_sharp` / `laser_cloud_less_sharp` | `sensor_msgs/PointCloud2` | 边特征（强 / 弱） |
| `laser_cloud_flat`  / `laser_cloud_less_flat`  | `sensor_msgs/PointCloud2` | 面特征（强 / 弱） |
| `cloud_info` | `cloud_msgs/cloud_info` | range image 元数据（`startRingIndex` / 地面 flag 等） |

不发 TF，让 FAST-LIO（`odom→base`）和 `pose_fusion`（`map→odom`）各管各的。

## 在系统中的角色

只在 **定位流程** 用：MCL 3DL 的 likelihood 模型按这 4 类特征分别打分（边对边、面对面），匹配比"对原始点云一锅端"更稳，对稀疏远点也更鲁棒。

## 主要参数

`dddnav_bringup/config/nav/base.yaml` 的 `mcl_ip` / `mcl_fa` 段：

| Key | 含义 |
|-----|------|
| `mcl_ip.imageProjection.stitcher_num` | 多少帧拼接后再投影。Mid360 单帧就够，设 1；老雷达 / 深度相机版可设 3 |
| `mcl_ip.imageProjection.ground_fov_*` | 地面提取的仰角窗口 |
| `mcl_fa.featureAssociation.{edge_threshold, surf_threshold}` | 边/面分类阈值 |
| `mcl_fa.featureAssociation.nearest_feature_search_distance` | 特征关联最近邻搜索半径 |

## 已知行为

`imageProjection` 与 `featureAssociation` 在发布特征时会用 `clock_->now()` 覆盖原雷达 stamp（继承自上游）。MCL 的近似时间同步是按这个行为调的，不要随便改，否则要重新调 MCL。
