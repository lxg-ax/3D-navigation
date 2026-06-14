// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <mpc_critics/toward_global_plan_model.h>

PLUGINLIB_EXPORT_CLASS(mpc_critics::TowardGlobalPlanModel, mpc_critics::ScoringModel)

namespace mpc_critics
{

TowardGlobalPlanModel::TowardGlobalPlanModel(){
  return;
  
}

void TowardGlobalPlanModel::onInitialize(){

  node_->declare_parameter(name_ + ".weight", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".weight", weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "weight: %.2f", weight_);

}


double TowardGlobalPlanModel::scoreTrajectory(base_trajectory::Trajectory &traj){

  if(shared_data_->pcl_prune_plan_->points.size()<3){
    RCLCPP_DEBUG(node_->get_logger().get_child(name_), "TowardGlobalPlanModel: size of prune plan is smaller than 3.");
    //@ if we return negative, the goal behavior might be strange (it is my guess, it should be justified)
    return 10.0;
  }
  
  geometry_msgs::msg::PoseStamped last_traj_pose = traj.getPoint(traj.getPointsSize()-1);

  pcl::KdTreeFLANN<pcl::PointXYZI> prune_plan_kdtree;
  prune_plan_kdtree.setInputCloud(shared_data_->pcl_prune_plan_);
  int K = 1;
  
  //@ get last traj pose
  pcl::PointXYZI pcl_traj_pose = traj.getPCLPoint(traj.getPointsSize()-1);
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);
  if ( prune_plan_kdtree.nearestKSearch (pcl_traj_pose, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
    return sqrt(pointNKNSquaredDistance[0]) * weight_;
  }
  else{
    return -12.0;
  }
  

}

}//end of name space