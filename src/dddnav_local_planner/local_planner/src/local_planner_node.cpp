// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <memory>
#include <perception_3d/perception_3d_ros.h>
#include <trajectory_generators/trajectory_generators_ros.h>
#include <mpc_critics/mpc_critics_ros.h>
#include <local_planner/local_planner.h>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv){

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto node_tg = std::make_shared<trajectory_generators::Trajectory_Generators_ROS>("trajectory_generators");
  auto node_mc = std::make_shared<mpc_critics::MPC_Critics_ROS>("mpc_critics");
  auto node_p3 = std::make_shared<perception_3d::Perception3D_ROS>("perception_3d_local");
  auto node_lp = std::make_shared<local_planner::Local_Planner>("local_planner");
 
  executor.add_node(node_tg);
  executor.add_node(node_mc);
  executor.add_node(node_p3);
  executor.add_node(node_lp);

  node_tg->initial();
  node_mc->initial();
  node_p3->initial();
  node_lp->initial(node_p3, node_mc, node_tg);
  executor.spin();


  rclcpp::shutdown();

}
