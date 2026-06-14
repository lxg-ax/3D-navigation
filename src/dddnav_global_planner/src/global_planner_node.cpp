// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <global_planner/dynamic_window_aware_global_planner.h>

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  auto perception_3d = std::make_shared<perception_3d::Perception3D_ROS>("perception_3d_global");
  auto global_planner = std::make_shared<global_planner::GlobalPlanner>("global_planner");
  auto dwa_global_planner = std::make_shared<global_planner::DWA_GlobalPlanner>("dynamic_window_aware_global_planner");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(perception_3d);
  perception_3d->initial();
  executor.add_node(global_planner);
  global_planner->initial(perception_3d);
  executor.add_node(dwa_global_planner);
  dwa_global_planner->initial(perception_3d, global_planner);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}