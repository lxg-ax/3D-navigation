// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <functional>
#include <memory>
#include <perception_3d/perception_3d_ros.h>

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);
  auto node = std::make_shared<perception_3d::Perception3D_ROS>("perception_3d");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  node->initial();
  executor.spin();

  return 0;
}