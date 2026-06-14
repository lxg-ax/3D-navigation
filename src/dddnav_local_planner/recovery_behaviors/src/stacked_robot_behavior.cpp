// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <recovery_behaviors/stacked_robot_behavior.h>

namespace recovery_behaviors
{


StackedRobotBehavior::StackedRobotBehavior(
                      const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger,
                      std::shared_ptr<tf2_ros::Buffer> m_tf2Buffer)
{
  logger_ = m_logger;
  shared_data_ = std::make_shared<recovery_behaviors::RecoveryBehaviorsSharedData>(m_tf2Buffer);
  access_ = new behavior_mutex_t();
  return;
}

StackedRobotBehavior::~StackedRobotBehavior()
{
  shared_data_.reset();
  delete access_;
}

/*
Plugin operations
*/
void StackedRobotBehavior::addPlugin(std::string behavior_name, std::shared_ptr<RobotBehavior> behavior)
{

  /*This is very important line that assign pointer to each model for shared data*/
  behavior->setSharedData(shared_data_);

  /*This is very important line that assign pointer to each behavior for shared data*/
  //model->setSharedData(shared_data_);
  robot_behaviors_.insert({behavior_name, behavior});
}

dddnav_sys_core::RecoveryState StackedRobotBehavior::runBehavior( 
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddnav_sys_core::action::RecoveryBehaviors>> goal_handle)
{
  
  const auto goal = goal_handle->get_goal();
  std::string behavior_name = goal->behavior_name;
  if (robot_behaviors_.find(behavior_name) == robot_behaviors_.end()){
    RCLCPP_ERROR(logger_->get_logger(), "%s not presented.", behavior_name.c_str());
    return dddnav_sys_core::RecoveryState::RECOVERY_BEHAVIOR_NOT_FOUND;
  }
  return robot_behaviors_[behavior_name]->runBehavior(goal_handle);

}

}//end of name space