# IMU 噪声参数：FAST-LIO vs LIO-SAM

两边读同一个 IMU，但参数命名、单位、语义不同，**不能复制粘贴**。

## 对照表

| 物理量 | FAST-LIO (`fastlio_mid360.yaml`) | LIO-SAM (`liosam_mid360.yaml`) |
|--------|----------------------------------|---------------------------------|
| 加速度计噪声 | `mapping.acc_cov` (方差 `(m/s²)²`) | `imuAccNoise` (标准差 `m/s²/√Hz`) |
| 陀螺仪噪声  | `mapping.gyr_cov` (方差 `(rad/s)²`) | `imuGyrNoise` (标准差 `rad/s/√Hz`) |
| acc bias 随机游走 | `mapping.b_acc_cov` | `imuAccBiasN` |
| gyr bias 随机游走 | `mapping.b_gyr_cov` | `imuGyrBiasN` |

FAST-LIO 直接把这个值当离散方差填进 EKF Q；LIO-SAM 把它平方后乘 I3 喂给 gtsam。
不要靠公式硬转换，**两边各自基于 datasheet 起一组、独立调到收敛**。

## MID360 datasheet 起点

- 加速度计噪声密度 `~0.24 mg/√Hz` ≈ `0.00235 m/s²/√Hz`
- 陀螺仪噪声密度  `~0.0067 °/s/√Hz` ≈ `1.17e-4 rad/s/√Hz`
- 实战常放大 10~50 倍以容忍标定与温漂

当前仓库值（FAST-LIO `acc_cov=0.1, gyr_cov=0.3, b_*=1e-4`；LIO-SAM `imuAccNoise=0.1, imuGyrNoise=0.01, imuAccBiasN=1e-3, imuGyrBiasN=1e-4`）就是这样独立调出来的，看着不像同值是正常的。
