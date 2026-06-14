// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <recovery_behaviors/robot_behavior.h>
#include <pluginlib/class_loader.hpp>
/*Debug*/
#include <sys/time.h>
#include <time.h>

/*To iterate the ymal for plugins*/



namespace recovery_behaviors
{

class StackedRobotBehavior{

  public:

    StackedRobotBehavior(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger, 
                          std::shared_ptr<tf2_ros::Buffer> m_tf2Buffer);
    ~StackedRobotBehavior();

    /*Plugin operations*/
    void addPlugin(std::string behavior_name, std::shared_ptr<recovery_behaviors::RobotBehavior> behavior);
    
    std::shared_ptr<recovery_behaviors::RecoveryBehaviorsSharedData> getSharedDataPtr(){return shared_data_;}

    // Provide a typedef to ease future code maintenance
    typedef std::recursive_mutex behavior_mutex_t;
    behavior_mutex_t* getMutex()
    {
      return access_;
    }
    
    dddnav_sys_core::RecoveryState runBehavior(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddnav_sys_core::action::RecoveryBehaviors>> goal_handle);

  private:

    std::map<std::string, std::shared_ptr<recovery_behaviors::RobotBehavior>> robot_behaviors_;
    /*mutex*/
    behavior_mutex_t* access_;

    std::shared_ptr<recovery_behaviors::RecoveryBehaviorsSharedData> shared_data_;
    
    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;

};

}//end of name space