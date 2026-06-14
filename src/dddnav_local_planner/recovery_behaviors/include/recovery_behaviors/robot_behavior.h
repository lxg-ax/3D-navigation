// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

/*Debug*/
#include <chrono>

#include <pluginlib/class_list_macros.hpp>
/*path for trajectory*/
#include <base_trajectory/trajectory.h>

/*For perception plugin*/
#include <perception_3d/perception_3d_ros.h>

/*For critics plugin*/
#include <mpc_critics/mpc_critics_ros.h>

/*For trajectory generators plugin*/
#include <trajectory_generators/trajectory_generators_ros.h>

/*For robot state*/
#include <recovery_behaviors/recovery_behaviors_shared_data.h>

/*For action server from move base*/
#include <dddnav_sys_core/dddnav_enum_states.h>

//@pass action server handle for plugin to abort
#include "rclcpp_action/rclcpp_action.hpp"
#include "dddnav_sys_core/action/recovery_behaviors.hpp"


namespace recovery_behaviors
{

class RobotBehavior{

  public:

    RobotBehavior();

    void initialize(const std::string name,
      const rclcpp::Node::WeakPtr& weak_node,
      std::shared_ptr<perception_3d::Perception3D_ROS> perception_3d,
      std::shared_ptr<mpc_critics::MPC_Critics_ROS> mpc_critics,
      std::shared_ptr<trajectory_generators::Trajectory_Generators_ROS> trajectory_generators
      );

    void setSharedData(std::shared_ptr<recovery_behaviors::RecoveryBehaviorsSharedData> shared_data);

    virtual dddnav_sys_core::RecoveryState runBehavior(
          const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddnav_sys_core::action::RecoveryBehaviors>> goal_handle) = 0;

  protected:

    virtual void onInitialize() = 0;
    
    rclcpp::Node::SharedPtr node_;

    std::string name_;

    //@ for percertion
    std::shared_ptr<perception_3d::Perception3D_ROS> perception_3d_ros_;
    //@ for critics
    std::shared_ptr<mpc_critics::MPC_Critics_ROS> mpc_critics_ros_;
    //@ for trajectory generator
    std::shared_ptr<trajectory_generators::Trajectory_Generators_ROS> trajectory_generators_ros_;
    
    std::shared_ptr<recovery_behaviors::RecoveryBehaviorsSharedData> shared_data_;

    bool terminate_by_recovery_behavior_ros_;

};


}//end of name space