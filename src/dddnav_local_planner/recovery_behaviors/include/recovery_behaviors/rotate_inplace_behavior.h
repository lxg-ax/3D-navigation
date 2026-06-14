// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <recovery_behaviors/robot_behavior.h>

/*For visualization*/
#include "geometry_msgs/msg/pose_array.hpp"

namespace recovery_behaviors
{

class RotateInPlaceBehavior: public RobotBehavior{

  public:
    
    RotateInPlaceBehavior();
    ~RotateInPlaceBehavior();

  private:

    void trans2Pose(geometry_msgs::msg::TransformStamped& trans, geometry_msgs::msg::PoseStamped& pose);
    double frequency_;
    double tolerance_;
    pcl::PointCloud<pcl::PointXYZ> cuboid_;
    void trajectory2posearray_cuboids(const base_trajectory::Trajectory& a_traj, 
                                      geometry_msgs::msg::PoseArray& pose_arr, pcl::PointCloud<pcl::PointXYZ>& cuboids_pcl);

    void getBestTrajectory(std::string traj_gen_name, base_trajectory::Trajectory& best_traj);

    dddnav_sys_core::RecoveryState runBehavior(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<dddnav_sys_core::action::RecoveryBehaviors>> goal_handle);
    
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_trajectory_pose_array_;
    
    rclcpp::Clock::SharedPtr clock_;
    std::string trajectory_generator_name_;

  protected:

    virtual void onInitialize();
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr aggregate_observation_;
    
    std::string robot_frame_, global_frame_;
    
    std::shared_ptr<std::vector<base_trajectory::Trajectory>> trajectories_;


};

}//end of name space
