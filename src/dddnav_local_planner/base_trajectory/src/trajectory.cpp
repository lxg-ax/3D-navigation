// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <base_trajectory/trajectory.h>

namespace base_trajectory {
  Trajectory::Trajectory()
    : xv_(0.0), yv_(0.0), thetav_(0.0), cost_(-1.0)
  {
  }

  Trajectory::Trajectory(double xv, double yv, double thetav, double time_delta, unsigned int num_pts)
    : xv_(xv), yv_(yv), thetav_(thetav), cost_(-1.0), time_delta_(time_delta)
  {
  }

  geometry_msgs::msg::PoseStamped Trajectory::getPoint(unsigned int index) const {
    return trajectory_path_.poses[index];
  }

  pcl::PointXYZI Trajectory::getPCLPoint(unsigned int index) const{
    return pcl_trajectory_path_.points[index];
  }

  pcl::PointCloud<pcl::PointXYZ> Trajectory::getCuboid(unsigned int index) const {
    return cuboids_[index];
  }

  cuboid_min_max_t Trajectory::getCuboidMinMax(unsigned int index) const {
    return cuboids_min_max_[index];
  }

  void Trajectory::setPoint(unsigned int index, double x, double y, double th){

  }

  bool Trajectory::addPoint(const geometry_msgs::msg::PoseStamped& pos, 
                            const pcl::PointCloud<pcl::PointXYZ>& cuboid,
                            const cuboid_min_max_t& cuboid_min_max){
    trajectory_path_.poses.push_back(pos);
    cuboids_.push_back(cuboid);
    cuboids_min_max_.push_back(cuboid_min_max);
    //@ for pcl
    pcl::PointXYZI ipt;
    ipt.x = pos.pose.position.x;
    ipt.y = pos.pose.position.y;
    ipt.z = pos.pose.position.z;
    ipt.intensity = 0.;
    pcl_trajectory_path_.push_back(ipt);

    return true;

  }

  void Trajectory::resetPoints(){
    trajectory_path_.poses.clear();
    cuboids_.clear();
    cuboids_min_max_.clear();
    pcl_trajectory_path_.points.clear();
    controls_.clear();
  }

  void Trajectory::getEndpoint(double& x, double& y, double& th) const {

  }

  unsigned int Trajectory::getPointsSize() const {
    return trajectory_path_.poses.size();
  }
};
