# dddnav_sys_core

底层共享类型 / 状态枚举 / 基类。纯支撑包，没有节点。

## 内容

* 多个包共用的状态枚举（FSM 状态、recovery 状态等）
* 通用基类（PluginBase 风格的接口）
* 跨包的常量 / 工具函数

## 在系统中的角色

`p2p_move_base`、`local_planner`、`recovery_behaviors`、`global_planner` 都引用这里的接口与枚举。改动之前先确认下游是否兼容。
