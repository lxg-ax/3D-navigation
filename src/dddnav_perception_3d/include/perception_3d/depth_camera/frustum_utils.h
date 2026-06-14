// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#ifndef PERCEPTION_3D_DEPTH_CAMERA_FRUSTUM_UTILS_H_
#define PERCEPTION_3D_DEPTH_CAMERA_FRUSTUM_UTILS_H_

#include <perception_3d/sensor.h>
#include <perception_3d/depth_camera/depth_camera_observation_buffer.hpp>
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace perception_3d
{

class FrustumUtils{

  public:
    void setObservationBuffers(std::map<std::string, std::shared_ptr<perception_3d::DepthCameraObservationBuffer>>& observation_buffers);
    bool isinFrustumsObservations(pcl::PointXYZI& testPoint);
    bool isAttachFRUSTUMs(pcl::PointXYZI& testPoint);
    bool isInsideFRUSTUMwoAttach(perception_3d::DepthCameraObservation& observation, pcl::PointXYZI& testPoint);
    bool isinFrustumsObservations(pcl::PointXYZ& testPoint);
    visualization_msgs::msg::MarkerArray current_frustums_marker_array_;
  private:
    std::map<std::string, std::shared_ptr<perception_3d::DepthCameraObservationBuffer>> observation_buffers_;
    

};

}//end of name space
#endif