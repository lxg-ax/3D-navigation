// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#ifndef MPC_CRITICS_SCORING_MODEL_H_
#define MPC_CRITICS_SCORING_MODEL_H_

#include "pluginlib/class_list_macros.hpp"

/*Debug*/
#include <chrono>
#include <mpc_critics/model_shared_data.h>


namespace mpc_critics
{

class ScoringModel{

  public:

    ScoringModel();

    void initialize(const std::string name, const rclcpp::Node::WeakPtr& weak_node);

    /**
     * return a score for trajectory traj
     */
    virtual double scoreTrajectory(base_trajectory::Trajectory &traj) = 0;

    void setSharedData(std::shared_ptr<mpc_critics::ModelSharedData> shared_data);

    std::string getModelName(){return name_;}

  protected:
    
    rclcpp::Node::SharedPtr node_;

    virtual void onInitialize() = 0;

    std::string name_;
    std::shared_ptr<mpc_critics::ModelSharedData> shared_data_;

    double weight_;


};


}//end of name space

#endif  // SCORING_MODEL_H