// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <trajectory_generators/stacked_generator.h>

namespace trajectory_generators
{


StackedGenerator::StackedGenerator(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger, 
                            std::shared_ptr<tf2_ros::Buffer> m_tf2Buffer)
{
  logger_ = m_logger;
  shared_data_ = std::make_shared<trajectory_generators::TrajectoryGeneratorSharedData>(m_tf2Buffer);
  access_ = new theory_mutex_t();
  return;
}

StackedGenerator::~StackedGenerator()
{

  std::map<std::string, std::shared_ptr<trajectory_generators::TrajectoryGeneratorTheory>>::iterator theory;
  for (theory = theories_.begin(); theory != theories_.end(); ++theory)
  {
    (*theory).second.reset();
  }
  shared_data_.reset();
  delete access_;
}

/*
Plugin operations
*/
void StackedGenerator::addPlugin(std::string pname, std::shared_ptr<TrajectoryGeneratorTheory> theory)
{
  theories_.insert(std::pair<std::string, std::shared_ptr<TrajectoryGeneratorTheory>>(pname, theory));
  theory->setSharedData(shared_data_);
}

void StackedGenerator::initializeTheories_wi_Shared_data() {

  //@initialise generator, basically reset the sample pivot in every theory
  std::map<std::string, std::shared_ptr<trajectory_generators::TrajectoryGeneratorTheory>>::iterator theory;
  for (theory = theories_.begin(); theory != theories_.end(); ++theory)
  {
    (*theory).second->initialise();
  }

}


/**
 * Create and return the next sample trajectory
 */
bool StackedGenerator::hasMoreTrajectories(std::string pname) {

  if(theories_.find( pname ) != theories_.end()){
    return theories_[pname]->hasMoreTrajectories();
  }
  else{
    RCLCPP_FATAL(logger_->get_logger(), "Trajectory plugin name is not in the theories.");
    return false;
  }

  
}

/**
 * Create and return the next sample trajectory
 */
bool StackedGenerator::nextTrajectory(std::string pname, base_trajectory::Trajectory& comp_traj) {

  if(theories_[pname]->hasMoreTrajectories())
  {
    if(theories_[pname]->nextTrajectory(comp_traj)){
      return true;
    }
    else
      return false;
  }
  else
    return false;

}

bool StackedGenerator::combineByScores(std::string pname,
                                       std::vector<base_trajectory::Trajectory>& scored,
                                       base_trajectory::Trajectory& combined){
  auto it = theories_.find(pname);
  if (it == theories_.end()){
    RCLCPP_FATAL(logger_->get_logger(), "combineByScores: unknown plugin %s", pname.c_str());
    return false;
  }
  return it->second->combineByScores(scored, combined);
}

}//end of name space