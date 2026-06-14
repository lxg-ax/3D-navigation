// Copyright (c) 2018, the mcl_3dl authors
// SPDX-License-Identifier: BSD-3-Clause

#ifndef MCL_3DL_LIDAR_MEASUREMENT_MODELS_LIDAR_MEASUREMENT_MODEL_LIKELIHOOD_H
#define MCL_3DL_LIDAR_MEASUREMENT_MODELS_LIDAR_MEASUREMENT_MODEL_LIKELIHOOD_H

#include "utilities.h"

#include <pcl/point_types.h>
#include <mcl_3dl/state_6dof.h>
#include <mcl_3dl/vec3.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <mcl_3dl/pf.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include "tf2/LinearMath/Transform.h"

namespace mcl_3dl
{

struct LidarMeasurementResult
{
  float likelihood;
  float quality;

  LidarMeasurementResult(const float likelihood_value, const float quality_value)
    : likelihood(likelihood_value)
    , quality(quality_value)
  {
  }
};

class LidarMeasurementModelLikelihood
{

private:

  rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameter_;

  int num_points_;
  int num_points_default_;
  int num_points_global_;

  int threshold_for_trusted_ground_;
  double radius_of_ground_search_;

  float match_dist_min_;
  float match_dist_flat_;

public:

  void loadConfig(
      const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger,
      const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr& m_parameter);
  void setGlobalLocalizationStatus(
      const int num_particles,
      const int current_num_particles);
  LidarMeasurementResult measure(
      pcl::KdTreeFLANN<mcl_3dl::pcl_t>& kdtree,
      pcl::KdTreeFLANN<mcl_3dl::pcl_t>& kdtree_ground,
      pcl::PointCloud<pcl::Normal>& normals,
      std::map<std::string, pcl::PointCloud<mcl_3dl::pcl_t>::Ptr> pcl_segmentations,
      const State6DOF& s) const;
};
}  // namespace mcl_3dl

#endif  // MCL_3DL_LIDAR_MEASUREMENT_MODELS_LIDAR_MEASUREMENT_MODEL_LIKELIHOOD_H
