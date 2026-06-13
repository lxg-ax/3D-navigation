# tuning/ 覆盖层

放现场调参用的小 overlay。原则：

1. **不复制完整 yaml**。只写需要覆盖的键。ROS 2 加载多个 yaml 时，后写覆盖前写。
2. **保留命名空间**。比如改 `mcl_3dl` 的参数必须以 `mcl_3dl:\n  ros__parameters:` 开头，否则不命中节点。
3. **每个 overlay 顶部写"为什么改"**。文件本身就是变更说明。

模板：

| 文件 | 用途 |
|------|------|
| `example_pose_fusion_overlay.yaml` | 室内慢速：ESKF 更信 LIO + 拒收门收紧 + 降级阈值收紧 |
| `example_mcl_overlay.yaml` | 长走廊 / 对称环境：粒子数门、上限、回落速率激进化 |
| `example_mppi_overlay.yaml` | 在 dd_simple 之外并行注册 MPPI generator + 一组对应 critic，调参看 dddnav_local_planner/README.md |

## 使用方式

ESKF 类（pose_fusion）：

```bash
ros2 launch dddnav_bringup localization.launch.py \
  pose_fusion_yaml:=$(pwd)/src/dddnav_bringup/config/reality/tuning/example_pose_fusion_overlay.yaml
```

导航 / MCL 类（叠在 nav profile 后）：

```bash
ros2 launch dddnav_bringup localization.launch.py \
  nav_profile:=$(pwd)/src/dddnav_bringup/config/reality/tuning/example_mcl_overlay.yaml
```

注：当前 `nav_profile` 还是单文件参数。要叠多层 overlay，先写一份合并好的 profile yaml 放到 `nav/`。

## 与默认 yaml 的关系

- 默认值在 `config/nav_base.yaml`、`pose_fusion.yaml`、`LIO-SAM/config/params_mid360.yaml`、`FAST_LIO/config/mid360_pc2.yaml`。
- 完整索引（参数在哪个文件、改这个的同时还要改什么）见 `../PARAMETERS.md`。
- **不要把现场 overlay 的值合并回默认 yaml**：默认值代表通用平台调参基线，overlay 代表特定现场的偏移。
