// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <mpc_critics/twirling_model.h>

PLUGINLIB_EXPORT_CLASS(mpc_critics::TwirlingModel, mpc_critics::ScoringModel)

namespace mpc_critics
{

TwirlingModel::TwirlingModel(){
  return;
  
}

void TwirlingModel::onInitialize(){

  node_->declare_parameter(name_ + ".weight", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".weight", weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "weight: %.2f", weight_);

}

double TwirlingModel::scoreTrajectory(base_trajectory::Trajectory &traj){

  return fabs(traj.thetav_) * weight_;

}

}//end of name space