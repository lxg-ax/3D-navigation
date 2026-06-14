// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <memory>
#include <perception_3d/perception_3d_ros.h>
#include <trajectory_generators/trajectory_generators_ros.h>
#include <mpc_critics/mpc_critics_ros.h>
#include <recovery_behaviors/recovery_behaviors_ros.h>
#include <local_planner/local_planner.h>
#include <p2p_move_base/p2p_global_plan_manager.h>
#include <p2p_move_base/p2p_move_base.h>


int main(int argc, char** argv){

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto node_tg = std::make_shared<trajectory_generators::Trajectory_Generators_ROS>("trajectory_generators");
  auto node_mc = std::make_shared<mpc_critics::MPC_Critics_ROS>("mpc_critics");
  auto node_p3 = std::make_shared<perception_3d::Perception3D_ROS>("perception_3d_local");
  auto node_rb = std::make_shared<recovery_behaviors::Recovery_Behaviors_ROS>("recovery_behaviors", node_p3, node_mc, node_tg);
  auto node_lp = std::make_shared<local_planner::Local_Planner>("local_planner");
  auto node_gpm = std::make_shared<p2p_move_base::P2PGlobalPlanManager>("global_plan_manager");
  auto node_p2p = std::make_shared<p2p_move_base::P2PMoveBase>("p2p_move_base");

  executor.add_node(node_tg);
  executor.add_node(node_mc);
  executor.add_node(node_p3);
  executor.add_node(node_lp);
  executor.add_node(node_rb);
  executor.add_node(node_gpm);
  executor.add_node(node_p2p);
  node_tg->initial();
  node_mc->initial();
  node_p3->initial();
  node_rb->initial();
  node_lp->initial(node_p3, node_mc, node_tg);
  node_gpm->initial();
  node_p2p->initial(node_lp, node_gpm);
  executor.spin();


  rclcpp::shutdown();

}
