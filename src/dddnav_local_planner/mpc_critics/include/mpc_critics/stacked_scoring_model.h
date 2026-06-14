// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#ifndef MPC_CRITICS_STACKED_SCORING_H_
#define MPC_CRITICS_STACKED_SCORING_H_

#include <mpc_critics/scoring_model.h>
/*Debug*/
#include <sys/time.h>
#include <time.h>

/*To iterate the ymal for plugins*/
#include <pluginlib/class_loader.hpp>

namespace mpc_critics
{

class StackedScoringModel{

  public:

    StackedScoringModel(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger, 
                        std::shared_ptr<tf2_ros::Buffer> m_tf2Buffer);
    ~StackedScoringModel();

    /*Plugin operations*/

    void addPluginByTraj(std::string traj_name, std::shared_ptr<ScoringModel> model);

    std::shared_ptr<mpc_critics::ModelSharedData> getSharedDataPtr(){return shared_data_;}

    void scoreTrajectory(std::string traj_gen_name, base_trajectory::Trajectory& one_traj);
    
    // Provide a typedef to ease future code maintenance
    typedef std::recursive_mutex model_mutex_t;
    model_mutex_t* getMutex()
    {
      return access_;
    }

  private:

    std::map<std::string, std::vector<std::shared_ptr<mpc_critics::ScoringModel>>> models_map_;
    std::shared_ptr<mpc_critics::ModelSharedData> shared_data_;
    /*mutex*/
    model_mutex_t* access_;

    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;

};

}//end of name space
#endif