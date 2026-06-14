// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#ifndef TRAJECTORY_GENERATOR_ROS_H_
#define TRAJECTORY_GENERATOR_ROS_H_

#include <trajectory_generators/stacked_generator.h>

namespace trajectory_generators
{

class Trajectory_Generators_ROS : public rclcpp::Node {
  
  public:
    Trajectory_Generators_ROS(const std::string& name);
    ~Trajectory_Generators_ROS();
    
    bool hasMoreTrajectories(std::string pname);
    bool nextTrajectory(std::string pname, base_trajectory::Trajectory& comp_traj);
    bool combineByScores(std::string pname,
                         std::vector<base_trajectory::Trajectory>& scored,
                         base_trajectory::Trajectory& combined);
    void initializeTheories_wi_Shared_data();

    std::shared_ptr<trajectory_generators::TrajectoryGeneratorSharedData> getSharedDataPtr(){return stacked_generator_->getSharedDataPtr();}

    StackedGenerator* getStackedGeneratorPtr(){return stacked_generator_;}  
    
    void initial();

  private:
    
    rclcpp::CallbackGroup::SharedPtr tf_listener_group_;
    /*For plugin loader*/
    pluginlib::ClassLoader<trajectory_generators::TrajectoryGeneratorTheory> theory_loader_;
    std::vector<std::string> plugins_;
    
  protected:

    StackedGenerator* stacked_generator_;

    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    std::shared_ptr<tf2_ros::Buffer> tf2Buffer_;  ///< @brief Used for transforming point clouds
    std::string name_;


};

}//end of name space
#endif
