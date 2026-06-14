// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <trajectory_generators/trajectory_generators_ros.h>

namespace trajectory_generators
{

Trajectory_Generators_ROS::Trajectory_Generators_ROS(const std::string& name):
  Node(name),
  name_(name),
  stacked_generator_(NULL),
  theory_loader_(name, "trajectory_generators::TrajectoryGeneratorTheory")
{

}

void Trajectory_Generators_ROS::initial(){


  //@Initialize transform listener and broadcaster
  tf_listener_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  tf2Buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface(),
    tf_listener_group_);
  tf2Buffer_->setCreateTimerInterface(timer_interface);
  tfl_ = std::make_shared<tf2_ros::TransformListener>(*tf2Buffer_);

  /*
  Start to load plugins
  */
  stacked_generator_ = new StackedGenerator(this->get_node_logging_interface(), tf2Buffer_);

  //@Start to load plugins
  this->declare_parameter("plugins", rclcpp::PARAMETER_STRING_ARRAY);
  rclcpp::Parameter plugins = this->get_parameter("plugins");
  plugins_ = plugins.as_string_array();
  for(auto i=plugins_.begin(); i!=plugins_.end(); i++){

    //@ get plugin type
    std::string get_plugin_type_str = (*i)+".plugin"; //plugins.plugin, ex: map.plugin
    this->declare_parameter(get_plugin_type_str, rclcpp::ParameterValue(""));
    rclcpp::Parameter plugin_type_param = this->get_parameter(get_plugin_type_str);
    std::string plugin_type_str = plugin_type_param.as_string();

    RCLCPP_INFO(this->get_logger(), "Use theory name: %s ---> %s", (*i).c_str(), plugin_type_str.c_str());  

    std::shared_ptr<trajectory_generators::TrajectoryGeneratorTheory> plugin = theory_loader_.createSharedInstance(plugin_type_str);
    stacked_generator_->addPlugin((*i), plugin);
    //Pass shared data for Sensor to share customized data betrween plugins
    plugin->initialize((*i), shared_from_this());
  }


}


Trajectory_Generators_ROS::~Trajectory_Generators_ROS()
{
  delete stacked_generator_;
}

/**
 * Create and return the next sample trajectory
 */
bool Trajectory_Generators_ROS::hasMoreTrajectories(std::string pname) {
  
  return stacked_generator_->hasMoreTrajectories(pname);
}

/**
 * Create and return the next sample trajectory
 */
bool Trajectory_Generators_ROS::nextTrajectory(std::string pname, base_trajectory::Trajectory& comp_traj) {
  return stacked_generator_->nextTrajectory(pname, comp_traj);
}

bool Trajectory_Generators_ROS::combineByScores(std::string pname,
                                                std::vector<base_trajectory::Trajectory>& scored,
                                                base_trajectory::Trajectory& combined){
  return stacked_generator_->combineByScores(pname, scored, combined);
}

void Trajectory_Generators_ROS::initializeTheories_wi_Shared_data() {

  stacked_generator_->initializeTheories_wi_Shared_data();

}

}//end of name space