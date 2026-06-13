// Copyright (c) 2024, DDDMobileRobot. BSD-3-Clause.
//
// Tiny helper that loads STDescManager's ConfigSetting from a rclcpp Node
// in one call. Replaces the upstream read_parameters(ros::NodeHandle&, ...)
// that we stripped during the ROS1 → ROS2 port. Param names match upstream.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include "dddnav_std_descriptor/STDesc.h"

namespace dddnav_std_descriptor
{

// Declare every STD parameter on `node` if not already declared, then read
// them into `cfg`. Defaults come from the upstream STDesc.cpp baseline.
inline void loadStdConfig(rclcpp::Node * node, ConfigSetting & cfg)
{
  auto pd = [&](const std::string & n, auto default_v) {
    if (!node->has_parameter(n))
      node->declare_parameter<decltype(default_v)>(n, default_v);
    return node->get_parameter(n).get_value<decltype(default_v)>();
  };

  // pre-process
  cfg.stop_skip_enable_     = pd("std.stop_skip_enable", 0);
  cfg.ds_size_              = pd("std.ds_size", 0.5);
  cfg.maximum_corner_num_   = pd("std.maximum_corner_num", 100);

  // key-point extraction
  cfg.plane_merge_normal_thre_ = pd("std.plane_merge_normal_thre", 0.1);
  cfg.plane_merge_dis_thre_    = pd("std.plane_merge_dis_thre", 0.3);
  cfg.plane_detection_thre_    = pd("std.plane_detection_thre", 0.01);
  cfg.voxel_size_              = pd("std.voxel_size", 2.0);
  cfg.voxel_init_num_          = pd("std.voxel_init_num", 10);
  cfg.proj_image_resolution_   = pd("std.proj_image_resolution", 0.5);
  cfg.proj_dis_min_            = pd("std.proj_dis_min", 0.0);
  cfg.proj_dis_max_            = pd("std.proj_dis_max", 2.0);
  cfg.corner_thre_             = pd("std.corner_thre", 10.0);

  // STD construction
  cfg.descriptor_near_num_       = pd("std.descriptor_near_num", 10);
  cfg.descriptor_min_len_        = pd("std.descriptor_min_len", 2.0);
  cfg.descriptor_max_len_        = pd("std.descriptor_max_len", 50.0);
  cfg.non_max_suppression_radius_ = pd("std.non_max_suppression_radius", 2.0);
  cfg.std_side_resolution_       = pd("std.std_side_resolution", 0.2);

  // place recognition
  cfg.skip_near_num_         = pd("std.skip_near_num", 50);
  cfg.candidate_num_         = pd("std.candidate_num", 50);
  cfg.sub_frame_num_         = pd("std.sub_frame_num", 10);
  cfg.rough_dis_threshold_   = pd("std.rough_dis_threshold", 0.03);
  cfg.vertex_diff_threshold_ = pd("std.vertex_diff_threshold", 0.7);
  cfg.icp_threshold_         = pd("std.icp_threshold", 0.5);
  cfg.normal_threshold_      = pd("std.normal_threshold", 0.2);
  cfg.dis_threshold_         = pd("std.dis_threshold", 0.5);
}

}  // namespace dddnav_std_descriptor
