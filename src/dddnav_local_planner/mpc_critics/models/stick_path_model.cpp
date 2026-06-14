// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <mpc_critics/stick_path_model.h>

PLUGINLIB_EXPORT_CLASS(mpc_critics::StickPathModel, mpc_critics::ScoringModel)

namespace mpc_critics
{

StickPathModel::StickPathModel(){
  return;
  
}

void StickPathModel::onInitialize(){

  node_->declare_parameter(name_ + ".weight", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".weight", weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "weight: %.2f", weight_);

}

double StickPathModel::scoreTrajectory(base_trajectory::Trajectory &traj){

  if(shared_data_->pcl_prune_plan_->points.size()<3){
    RCLCPP_DEBUG(node_->get_logger().get_child(name_), "StickPathModel: size of prune plan is smaller than 3.");
    //@ if we return negative, the goal behavior might be strange (it is my guess, it should be justified)
    return 10.0;
  }
  pcl::KdTreeFLANN<pcl::PointXYZI> prune_plan_kdtree;
  prune_plan_kdtree.setInputCloud(shared_data_->pcl_prune_plan_);
  int K = 1;
  double normalized_distance = 0.0;
  for(unsigned int i=0;i<traj.getPointsSize();i++){

    pcl::PointXYZI pcl_traj_pose = traj.getPCLPoint(i);
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    if ( prune_plan_kdtree.nearestKSearch (pcl_traj_pose, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
      normalized_distance += sqrt(pointNKNSquaredDistance[0]);
    }
    else{
      normalized_distance += 3.0;
    }
  }
  normalized_distance /= shared_data_->pcl_prune_plan_->points.size();
  //RCLCPP_INFO(this->get_logger(), "Normalized_distance: %f",normalized_distance);
  return normalized_distance;
}

}//end of name space