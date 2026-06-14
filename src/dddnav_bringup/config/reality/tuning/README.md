# tuning/ 覆盖层

现场调参的 overlay 模板。**不要复制完整 yaml**，只写要覆盖的键，ROS 2 后写覆盖前写。

| 文件 | 用途 |
|------|------|
| `example_pose_fusion_overlay.yaml` | 室内慢速：ESKF 更信 LIO + 拒收门收紧 |
| `example_mcl_overlay.yaml` | 长走廊/对称环境：粒子数激进化 |
| `example_mppi_overlay.yaml` | 在 dd_simple 之外并行注册 MPPI generator |

## 用法

```bash
# pose_fusion 类
ros2 launch dddnav_bringup localization.launch.py \
  pose_fusion_yaml:=$(pwd)/src/dddnav_bringup/config/reality/tuning/example_pose_fusion_overlay.yaml

# nav/MCL 类
ros2 launch dddnav_bringup localization.launch.py \
  nav_profile:=$(pwd)/src/dddnav_bringup/config/reality/tuning/example_mcl_overlay.yaml
```

参数完整索引见 `../../PARAMETERS.md`。**不要把 overlay 值合并回默认 yaml**——默认是基线，overlay 是现场偏移。
