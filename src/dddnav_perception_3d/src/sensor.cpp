// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <perception_3d/sensor.h>

namespace perception_3d
{

Sensor::Sensor(){
  return;
}

void Sensor::setSharedData(std::shared_ptr<perception_3d::SharedData> shared_data){
  shared_data_ = shared_data;
}

void Sensor::initialize(std::string name,
        const rclcpp::Node::WeakPtr& weak_node,
        const std::shared_ptr<perception_3d::GlobalUtils>& gbl_utils){
    
  node_ = weak_node.lock();
  name_ = name;
  gbl_utils_ = gbl_utils;
  shared_data_->dgraph_update_request_[name_] = true;
  sensor_current_observation_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  expected_sensor_time_ = 0.1;
  current_ = false;
  onInitialize();
  opinion_ = perception_3d::PASS;
}

unsigned long Sensor::get_dGraphSize(){
  return dGraph_.getdGraphSize();
}

}//end of name space