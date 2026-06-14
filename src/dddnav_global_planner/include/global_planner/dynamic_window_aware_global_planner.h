// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"

/*global planner*/
#include <global_planner/global_planner.h>

/*For edge markers*/
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

/*For distance calculation*/
#include <pcl/common/geometry.h>
#include <math.h>

/*RANSAC*/
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

/*TF listener, although it is included in sensor.h*/
#include "tf2_ros/buffer.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/create_timer_ros.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/time.h"
#include <geometry_msgs/msg/transform_stamped.hpp>

/*srv msg for make plan*/
#include "dddnav_sys_core/action/get_plan.hpp"
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace global_planner
{

class DWA_GlobalPlanner : public rclcpp::Node {
    public:
      DWA_GlobalPlanner(const std::string& name);
      ~DWA_GlobalPlanner();

      void initial(const std::shared_ptr<perception_3d::Perception3D_ROS>& perception_3d, 
                    const std::shared_ptr<global_planner::GlobalPlanner>& global_planner);
   
    private:

      bool is_active(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddnav_sys_core::action::GetPlan>> handle) const
      {
        return handle != nullptr && handle->is_active();
      }

      rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const dddnav_sys_core::action::GetPlan::Goal> goal);

      rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddnav_sys_core::action::GetPlan>> goal_handle);

      void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddnav_sys_core::action::GetPlan>> goal_handle);
      
      std::shared_ptr<rclcpp_action::ServerGoalHandle<dddnav_sys_core::action::GetPlan>> current_handle_;

      rclcpp_action::Server<dddnav_sys_core::action::GetPlan>::SharedPtr action_server_global_planner_;

      rclcpp::CallbackGroup::SharedPtr action_server_group_;
      
      rclcpp::Clock::SharedPtr clock_;
      rclcpp::TimerBase::SharedPtr threading_timer_;
      
      rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;
      
      std::shared_ptr<perception_3d::Perception3D_ROS> perception_3d_ros_;
      std::shared_ptr<global_planner::GlobalPlanner> global_planner_;

      std::string global_frame_;
      std::string robot_frame_;
      geometry_msgs::msg::PoseStamped new_goal_;
      geometry_msgs::msg::PoseStamped current_goal_;
      nav_msgs::msg::Path global_path_;
      nav_msgs::msg::Path global_dwa_path_;
      double look_ahead_distance_;
      double recompute_frequency_;
      
      pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_global_path_; 
      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_global_path_;
      
      /*Func*/
      void makePlan(const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddnav_sys_core::action::GetPlan>> goal_handle);
      bool isNewGoal();
      void determineDWAPlan();

};

}
