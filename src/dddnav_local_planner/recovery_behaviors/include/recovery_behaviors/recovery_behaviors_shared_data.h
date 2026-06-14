// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#ifndef RECOVERY_BEHAVIORS_SHARED_DATA_H_
#define RECOVERY_BEHAVIORS_SHARED_DATA_H_

#include "rclcpp/rclcpp.hpp"

/*TF listener*/
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/impl/utils.h>
#include <tf2/time.h>

/*For tf2::matrix3x3 as quaternion to euler*/
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

/*For robot state*/
#include "nav_msgs/msg/odometry.hpp"

#include <pcl/point_cloud.h>
#include <angles/angles.h>

namespace recovery_behaviors
{

class RecoveryBehaviorsSharedData{
  public:

    RecoveryBehaviorsSharedData(std::shared_ptr<tf2_ros::Buffer> m_tf2Buffer):tf2Buffer_(m_tf2Buffer){};
    
    std::shared_ptr<tf2_ros::Buffer> tf2Buffer(){return tf2Buffer_;}

    geometry_msgs::msg::TransformStamped robot_pose_;

    std::string global_frame_, base_frame_;

    nav_msgs::msg::Odometry robot_state_;

  private:

    std::shared_ptr<tf2_ros::Buffer> tf2Buffer_; 
    
    
};


}//end of name space

#endif  // RECOVERY_BEHAVIORS_SHARED_DATA_H