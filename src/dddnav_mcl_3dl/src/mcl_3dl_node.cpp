// Copyright (c) 2016-2020, the mcl_3dl authors
// SPDX-License-Identifier: BSD-3-Clause

#include <memory>
#include <mcl_3dl.h>
#include <mcl_3dl/sub_maps.h>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{

  rclcpp::init(argc, argv);
  auto node = std::make_shared<mcl_3dl::MCL3dlNode>("mcl_3dl");
  auto node_sub_maps = std::make_shared<mcl_3dl::SubMaps>("sub_maps");
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(node_sub_maps);
  node->configure(node_sub_maps);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
