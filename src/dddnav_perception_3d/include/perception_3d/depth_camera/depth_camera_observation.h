// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#ifndef DEPTH_CAMERA_OBSERVATION_H_
#define DEPTH_CAMERA_OBSERVATION_H_

#include <geometry_msgs/msg/point.hpp> 
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

namespace perception_3d
{

class PointCloudCluster{

  public:
    pcl::PointCloud<pcl::PointXYZI> point_cloud_;
    pcl::PointXYZ centroid_;
};

class DepthCameraObservation
{
public:

  DepthCameraObservation(const sensor_msgs::msg::PointCloud2& cloud);
  //DepthCameraObservation(const DepthCameraObservation& obs);

  virtual ~DepthCameraObservation();

  pcl::PointXYZ getVec(pcl::PointXYZ vec1, pcl::PointXYZ vec2);
  pcl::PointXYZ getCrossProduct(pcl::PointXYZ vec1, pcl::PointXYZ vec2);
  void getPlaneN(Eigen::Vector4f& plane_equation, pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3);
  void findFrustumVertex();
  void findFrustumNormal();
  void findFrustumPlane();



  geometry_msgs::msg::Point origin_;
  //@ cloud_ is in global frame after min/max height check and distance check
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_;
  //@ raw_cloud_ is in sensor frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud_;
  //@ everything related to frustum is in global frame
  pcl::PointCloud<pcl::PointXYZ>::Ptr frustum_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr frustum_normal_;
  std::vector<Eigen::Vector4f> frustum_plane_equation_;
  //@ clusters storage
  std::vector<perception_3d::PointCloudCluster> clusters_;

  double FOV_V_;
  double FOV_W_;
  double min_detect_distance_;
  double max_detect_distance_;

  pcl::PointXYZ BRNear_;
  pcl::PointXYZ TLFar_;
  

};
}
#endif  // DEPTH_CAMERA_OBSERVATION_H_
