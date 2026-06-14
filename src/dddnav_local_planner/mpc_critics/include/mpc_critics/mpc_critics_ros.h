// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#ifndef MPC_CRITICS_ROS_H_
#define MPC_CRITICS_ROS_H_

#include <mpc_critics/stacked_scoring_model.h>

namespace mpc_critics
{

class MPC_Critics_ROS : public rclcpp::Node {
  
  public:

    MPC_Critics_ROS(const std::string& name);
    ~MPC_Critics_ROS();
    
    void initial();

    void updateSharedData(); 
    
    void scoreTrajectory(std::string traj_gen_name, base_trajectory::Trajectory& one_traj);

    StackedScoringModel* getStackedScoringModelPtr(){return stacked_scoring_model_;}  

    std::shared_ptr<mpc_critics::ModelSharedData> getSharedDataPtr(){return stacked_scoring_model_->getSharedDataPtr();}

  private:
    
    rclcpp::CallbackGroup::SharedPtr tf_listener_group_;
    /*For plugin loader*/
    pluginlib::ClassLoader<mpc_critics::ScoringModel> model_loader_;
    std::vector<std::string> plugins_;
    
  protected:

    StackedScoringModel* stacked_scoring_model_;

    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    std::shared_ptr<tf2_ros::Buffer> tf2Buffer_;  ///< @brief Used for transforming point clouds
    std::string name_;


};

}//end of name space
#endif