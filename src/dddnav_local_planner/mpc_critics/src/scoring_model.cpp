// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <mpc_critics/scoring_model.h>

namespace mpc_critics
{

ScoringModel::ScoringModel(){

}

void ScoringModel::initialize(const std::string name, const rclcpp::Node::WeakPtr& weak_node){
  name_ = name;
  node_ = weak_node.lock();
  weight_ = 0;
  onInitialize();
}

void ScoringModel::setSharedData(std::shared_ptr<mpc_critics::ModelSharedData> shared_data){
  shared_data_ = shared_data;
}


}//end of name space