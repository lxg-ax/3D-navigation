// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include "perception_3d/shared_data.h"

namespace perception_3d
{

SharedData::SharedData(){
  is_static_layer_ready_ = false;
  static_map_size_ = 0;
  static_ground_size_ = 0;
  aggregate_observation_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  current_allowed_max_linear_speed_ = -1.0;
}

void SharedData::requestAllLayersToResetDGraph(){
  for(auto i=dgraph_update_request_.begin();i!=dgraph_update_request_.end();i++){
    (*i).second = true;
  }
}

bool SharedData::isAllLayersBeenReset(){
  for(auto i=dgraph_update_request_.begin();i!=dgraph_update_request_.end();i++){
    if((*i).second)
      return false;
  }
  return true;
}

}//end of name space