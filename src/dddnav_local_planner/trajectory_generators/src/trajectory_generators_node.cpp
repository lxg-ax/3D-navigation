// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <memory>
#include <trajectory_generators/trajectory_generators_ros.h>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  auto node = std::make_shared<trajectory_generators::Trajectory_Generators_ROS>("trajectory_generators");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  node->initial();
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
