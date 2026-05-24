# cloud_msgs

`dddnav_mcl_feature` 用的自定义点云消息（移植自 LeGO-LOAM 的 `cloud_info`）。纯接口包，没有节点。

## 消息

| Msg | 字段 | 用途 |
|-----|------|------|
| `cloud_info` | `startRingIndex`、`endRingIndex`、`pointColInd`、`pointRange`、`groundFlag` 等 | 点云 range image 的 metadata，给特征关联节点（`featureAssociation`）用 |

## 在系统中的角色

`mcl_feature` 的 `imageProjection` → `featureAssociation` 之间靠这个消息传递每个点的环号 / 起止索引 / 是否地面。MCL 3DL 的 likelihood 模型按这套元信息做分类匹配，所以接口必须保持稳定。
