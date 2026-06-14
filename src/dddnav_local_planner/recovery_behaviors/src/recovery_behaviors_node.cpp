// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <memory>
#include <perception_3d/perception_3d_ros.h>
#include <trajectory_generators/trajectory_generators_ros.h>
#include <mpc_critics/mpc_critics_ros.h>
#include <recovery_behaviors/recovery_behaviors_ros.h>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto node_tg = std::make_shared<trajectory_generators::Trajectory_Generators_ROS>("trajectory_generators");
  auto node_mc = std::make_shared<mpc_critics::MPC_Critics_ROS>("mpc_critics");
  auto node_p3 = std::make_shared<perception_3d::Perception3D_ROS>("perception_3d");
  auto node_rb = std::make_shared<recovery_behaviors::Recovery_Behaviors_ROS>("recovery_behaviors", node_p3, node_mc, node_tg);
  executor.add_node(node_tg);
  executor.add_node(node_mc);
  executor.add_node(node_p3);
  executor.add_node(node_rb);
  node_tg->initial();
  node_mc->initial();
  node_p3->initial();
  node_rb->initial();
  executor.spin();


  rclcpp::shutdown();

  return 0;
}
