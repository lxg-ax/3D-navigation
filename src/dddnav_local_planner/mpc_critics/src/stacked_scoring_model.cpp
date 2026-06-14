// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <mpc_critics/stacked_scoring_model.h>

namespace mpc_critics
{


StackedScoringModel::StackedScoringModel(
                              const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger,
                              std::shared_ptr<tf2_ros::Buffer> m_tf2Buffer)
{
  logger_ = m_logger;
  shared_data_ = std::make_shared<mpc_critics::ModelSharedData>(m_tf2Buffer);
  access_ = new model_mutex_t();
  return;
}

StackedScoringModel::~StackedScoringModel()
{
  shared_data_.reset();
  delete access_;
}

/*
Plugin operations
*/

void StackedScoringModel::addPluginByTraj(std::string traj_name, std::shared_ptr<ScoringModel> model)
{

  /*This is very important line that assign pointer to each model for shared data*/
  model->setSharedData(shared_data_);

  if ( models_map_.find(traj_name) == models_map_.end() ) {
    // not found
    std::vector<std::shared_ptr<mpc_critics::ScoringModel>> models;
    models_map_.insert({traj_name, models});
    models_map_[traj_name].push_back(model);
  } else {
    // found
    models_map_[traj_name].push_back(model);
  }

}

void StackedScoringModel::scoreTrajectory(std::string traj_gen_name, base_trajectory::Trajectory& one_traj){
  
  one_traj.rejected_by_ = "pass";

  for (std::vector<std::shared_ptr<ScoringModel> >::iterator model =  models_map_[traj_gen_name].begin(); model !=  models_map_[traj_gen_name].end();
       ++model)
  {
    
    RCLCPP_DEBUG(logger_->get_logger(), "Rate trajectory by critics: %s", (*model)->getModelName().c_str());
    double return_cost = (*model)->scoreTrajectory(one_traj);
    if(return_cost<0){
      one_traj.cost_ = return_cost;
      one_traj.rejected_by_ = (*model)->getModelName();
      break;
    }
    else{
      one_traj.cost_ += return_cost;
    }

  }

}




}//end of name space