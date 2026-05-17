# cloud_msgs

Custom point-cloud message definitions used by `dddnav_mcl_feature` (port of LeGO-LOAM's `cloud_info`).

## Messages

| Msg | Used by |
|-----|---------|
| `cloud_info` | `dddnav_mcl_feature` `imageProjection` → `featureAssociation` (range image metadata, ring/start/end indices, ground flags) |

Pure interface package, no nodes.
