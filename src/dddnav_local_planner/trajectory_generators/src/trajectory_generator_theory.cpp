// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <trajectory_generators/trajectory_generator_theory.h>

#include <limits>

namespace trajectory_generators
{

TrajectoryGeneratorTheory::TrajectoryGeneratorTheory(){

}

void TrajectoryGeneratorTheory::initialize(const std::string name, const rclcpp::Node::WeakPtr& weak_node){
  name_ = name;
  node_ = weak_node.lock();
  onInitialize();
}

void TrajectoryGeneratorTheory::setSharedData(std::shared_ptr<trajectory_generators::TrajectoryGeneratorSharedData> shared_data){
  shared_data_ = shared_data;
}

bool TrajectoryGeneratorTheory::combineByScores(
    std::vector<base_trajectory::Trajectory>& scored,
    base_trajectory::Trajectory& combined){
  combined.cost_ = -1.0;
  double min_cost = std::numeric_limits<double>::max();
  bool found = false;
  for (auto& t : scored){
    if (t.cost_ >= 0.0 && t.cost_ <= min_cost){
      combined = t;
      min_cost = t.cost_;
      found = true;
    }
  }
  return found;
}


}//end of name space