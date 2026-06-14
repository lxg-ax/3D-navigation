// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#ifndef PERCEPTION_3D_GLOBAL_UTILS_H_
#define PERCEPTION_3D_GLOBAL_UTILS_H_

/*TF listener*/
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/create_timer_ros.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/time.h"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

namespace perception_3d
{

class GlobalUtils{
  public:
    GlobalUtils(std::string m_global_frame, 
      std::string m_robot_base_frame, 
      double max_obstacle_distance, 
      double inscribed_radius,
      double inflation_descending_rate,
      double inflation_radius,
      std::shared_ptr<tf2_ros::Buffer> m_tf2Buffer):

      global_frame_(m_global_frame),
      robot_base_frame_(m_robot_base_frame),
      max_obstacle_distance_(max_obstacle_distance), 
      inscribed_radius_(inscribed_radius),
      inflation_descending_rate_(inflation_descending_rate),
      inflation_radius_(inflation_radius),
      tf2Buffer_(m_tf2Buffer){};

    std::string getGblFrame(){return global_frame_;}
    std::string getRobotFrame(){return robot_base_frame_;}
    double getMaxObstacleDistance(){return max_obstacle_distance_;}
    double getInscribedRadius(){return inscribed_radius_;}
    double getInflationDescendingRate(){return inflation_descending_rate_;}
    double getInflationRadius(){return inflation_radius_;}
    
    std::shared_ptr<tf2_ros::Buffer> tf2Buffer(){return tf2Buffer_;}

  private:
    std::string global_frame_;
    std::string robot_base_frame_;
    std::shared_ptr<tf2_ros::Buffer> tf2Buffer_; 
    double max_obstacle_distance_;
    double inscribed_radius_;
    double inflation_descending_rate_;
    double inflation_radius_;
};


}//end of name space

#endif