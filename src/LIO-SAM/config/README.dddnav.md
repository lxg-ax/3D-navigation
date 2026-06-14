# 此目录的 yaml 不再生效

LIO-SAM 调参在 `src/dddnav_bringup/config/reality/slam/liosam_mid360.yaml`。
本目录的文件是上游 TixiaoShan/LIO-SAM 默认值（含本仓 STD 字段补充），仅作同步参考，改了无效。

IMU 噪声段与 FAST-LIO 互相影响，对照见 `reality/slam/IMU_NOTES.md`。

CLI 临时换：`lio_sam_config:=/abs/path/to/your.yaml`。
