// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#ifndef P2P_GLOBAL_PLAN_MANAGER_H
#define P2P_GLOBAL_PLAN_MANAGER_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2/time.h"
#include "tf2/transform_datatypes.h"
#include "tf2/utils.h"

#include "dddnav_sys_core/action/get_plan.hpp"

#include <mutex>

// chrono_literals handles user-defined time durations (e.g. 500ms) 
using namespace std::chrono_literals;

namespace p2p_move_base
{
class P2PGlobalPlanManager  : public rclcpp::Node 
{

private:
  
  std::string name_;
  rclcpp::Clock::SharedPtr clock_;
  std::mutex access_;

  std::shared_ptr<tf2_ros::TransformListener> tfl_;
  std::shared_ptr<tf2_ros::Buffer> tf2Buffer_;
  
  std::string global_planner_action_name_;
  double global_plan_query_frequency_;
  geometry_msgs::msg::PoseStamped goal_;
  bool is_planning_;
  bool got_first_goal_;
  nav_msgs::msg::Path global_path_;

  rclcpp::CallbackGroup::SharedPtr tf_listener_group_;
  rclcpp::CallbackGroup::SharedPtr timer_group_;
  rclcpp::CallbackGroup::SharedPtr global_planner_client_group_;
  
  rclcpp::TimerBase::SharedPtr loop_timer_;

  rclcpp_action::Client<dddnav_sys_core::action::GetPlan>::SharedPtr global_planner_client_ptr_;
  void global_planner_client_goal_response_callback(const rclcpp_action::ClientGoalHandle<dddnav_sys_core::action::GetPlan>::SharedPtr & goal_handle);
  void global_planner_client_result_callback(const rclcpp_action::ClientGoalHandle<dddnav_sys_core::action::GetPlan>::WrappedResult & result);
  

public:

  P2PGlobalPlanManager(std::string name);
  ~P2PGlobalPlanManager();
  
  void queryThread();

  void initial();
  void setGoal(const geometry_msgs::msg::PoseStamped& goal);
  void resume();
  void stop();
  bool hasPlan();
  void copyPlan(std::vector<geometry_msgs::msg::PoseStamped>& plan);

};
}  // namespace p2p_move_base

#endif  // P2P_GLOBAL_PLAN_MANAGER_H
