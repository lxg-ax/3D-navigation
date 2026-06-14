# 此目录的 yaml 不再生效

FAST-LIO 调参在 `src/dddnav_bringup/config/reality/slam/fastlio_mid360.yaml`。
本目录的文件是上游 hku-mars/FAST_LIO 默认值，仅作同步参考，改了无效。

IMU 噪声段与 LIO-SAM 互相影响，对照见 `reality/slam/IMU_NOTES.md`。

CLI 临时换：`fastlio_config:=/abs/path/to/your.yaml`。
