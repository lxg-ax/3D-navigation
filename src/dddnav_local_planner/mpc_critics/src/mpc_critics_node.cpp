// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <memory>
#include <mpc_critics/mpc_critics_ros.h>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  auto node = std::make_shared<mpc_critics::MPC_Critics_ROS>("mpc_critics");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  node->initial();
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
