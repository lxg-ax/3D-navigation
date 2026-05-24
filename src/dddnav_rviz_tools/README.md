# dddnav_rviz_tools

RViz 自定义面板与默认插件集合。提升地图编辑和位姿图调试效率。

## 插件

| 名称 | 作用 |
|------|------|
| `map_editor_panel` | PCD 地图编辑：切片、删点、保存 |
| `mapping_panel` | 建图过程的可视化与控制（启动 / 暂停 / 保存） |
| `pose_graph_editor_panel` | 单条位姿图编辑（删关键帧、加约束、重新优化） |
| `pose_graph_merge_editor_panel` | 多条位姿图融合（拼楼层 / 拼区域） |
| `dddnav_rviz_default_plugins` | 项目里复用的常用 display 插件集合 |

## 在系统中的角色

不是运行时必需，但建图调试 / 多楼层拼接时几乎不可缺。RViz 配置（`dddnav_bringup/rviz/`）默认会加载这些面板。
