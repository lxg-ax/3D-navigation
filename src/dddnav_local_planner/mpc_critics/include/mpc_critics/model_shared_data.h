// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#ifndef MPC_CRITICS_MODEL_SHARED_DATA_H_
#define MPC_CRITICS_MODEL_SHARED_DATA_H_

#include "rclcpp/rclcpp.hpp"

/*TF listener*/
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/create_timer_ros.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/time.h"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>

/*path for trajectory*/
#include <base_trajectory/trajectory.h>
/*For tf2::matrix3x3 as quaternion to euler*/
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

/*For robot state*/
#include "nav_msgs/msg/odometry.hpp"

#include <pcl/point_cloud.h>

/*kdtree*/
#include <pcl/kdtree/kdtree_flann.h>

//@tf2::eigenToTransform
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Core>

namespace mpc_critics
{

class ModelSharedData{
  public:

    ModelSharedData(std::shared_ptr<tf2_ros::Buffer> m_tf2Buffer):tf2Buffer_(m_tf2Buffer){};
    
    std::shared_ptr<tf2_ros::Buffer> tf2Buffer(){return tf2Buffer_;}

    void updateData(){
      global_frame_ = robot_pose_.header.frame_id;
      base_frame_ = robot_pose_.child_frame_id;
      /* Critics get kd tree generated in perception_ros when calling aggregateObservations in local planner*/
      if(pcl_perception_->points.size()>=5){
        pcl_perception_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
        pcl_perception_kdtree_->setInputCloud(pcl_perception_);        
      }
      
      pcl_prune_plan_.reset(new pcl::PointCloud<pcl::PointXYZI>);
      for(auto i=prune_plan_.poses.begin();i!=prune_plan_.poses.end();i++){
        pcl::PointXYZI ipt;
        ipt.x = (*i).pose.position.x;
        ipt.y = (*i).pose.position.y;
        ipt.z = (*i).pose.position.z;
        ipt.intensity = 0.;
        pcl_prune_plan_->push_back(ipt);
      }
    }
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr pcl_perception_kdtree_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_perception_;
    
    //@ this will be easy use for kdtree from pct
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_prune_plan_;

    nav_msgs::msg::Path prune_plan_;

    geometry_msgs::msg::TransformStamped robot_pose_;

    std::string global_frame_, base_frame_;

    nav_msgs::msg::Odometry robot_state_;

    double heading_deviation_;

  private:

    std::shared_ptr<tf2_ros::Buffer> tf2Buffer_; 
    
    

};


}//end of name space

#endif  // MODEL_SHARED_DATA_H