# dddnav_cloud_msgs

dddnav 内部自定义点云消息。纯接口包，无节点。源自 LeGO-LOAM 的 `cloud_info`。

## 功能

为 `dddnav_mcl_feature` 的 `imageProjection` → `featureAssociation` 流水线传递 range image 元数据。

## 在导航栈中的角色

`mcl_feature` 在做点云分类（地面 / 边特征 / 面特征）时把每个点的环号、起止列、距离、地面 flag 打包进 `CloudInfo`，下游 `featureAssociation` 直接按这套元信息匹配，不用重新算 range image。`dddnav_mcl_3dl` 间接依赖（package.xml 声明），但当前 likelihood 模型不直接读这个消息。

## 消息

| Msg | 关键字段 |
|-----|---------|
| `CloudInfo` | `startRingIndex` / `endRingIndex` / `segmentedCloudColInd` / `segmentedCloudRange` / `segmentedCloudGroundFlag` 等 |

## 依赖

`std_msgs`、`builtin_interfaces`。
