// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <recovery_behaviors/robot_behavior.h>

namespace recovery_behaviors
{

RobotBehavior::RobotBehavior(){

}

void RobotBehavior::initialize(const std::string name,
      const rclcpp::Node::WeakPtr& weak_node,
      std::shared_ptr<perception_3d::Perception3D_ROS> perception_3d,
      std::shared_ptr<mpc_critics::MPC_Critics_ROS> mpc_critics,
      std::shared_ptr<trajectory_generators::Trajectory_Generators_ROS> trajectory_generators){
  
  node_ = weak_node.lock();
  perception_3d_ros_ = perception_3d;
  mpc_critics_ros_ = mpc_critics;
  trajectory_generators_ros_ = trajectory_generators;
  name_ = name;
  onInitialize();
}

void RobotBehavior::setSharedData(std::shared_ptr<recovery_behaviors::RecoveryBehaviorsSharedData> shared_data){
  shared_data_ = shared_data;
}

}//end of name space