// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <mpc_critics/shortest_angle_model.h>

PLUGINLIB_EXPORT_CLASS(mpc_critics::ShortestAngleModel, mpc_critics::ScoringModel)

namespace mpc_critics
{

ShortestAngleModel::ShortestAngleModel(){
  return;
  
}

void ShortestAngleModel::onInitialize(){

  node_->declare_parameter(name_ + ".weight", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".weight", weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "weight: %.2f", weight_);

}

double ShortestAngleModel::scoreTrajectory(base_trajectory::Trajectory &traj){

  RCLCPP_DEBUG(node_->get_logger().get_child(name_), "Heading deviation: %.2f", shared_data_->heading_deviation_);
  double weight;
  if(shared_data_->heading_deviation_>=0){
    if(traj.thetav_>=0)
      weight = weight_;
    else
      weight = weight_ * 2;
  }
  else{
    if(traj.thetav_>=0)
      weight = weight_ * 2;
    else
      weight = weight_;  
  }
  RCLCPP_DEBUG(node_->get_logger().get_child(name_), "Heading deviation: %.2f, traj theta_v: %.2f, weight: %.2f", shared_data_->heading_deviation_, traj.thetav_, weight);
  return weight;
}

}//end of name space