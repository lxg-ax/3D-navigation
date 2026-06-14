// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <perception_3d/path_blocked_strategy.h>

PLUGINLIB_EXPORT_CLASS(perception_3d::PathBlockedStrategy, perception_3d::Sensor)

namespace perception_3d
{

PathBlockedStrategy::PathBlockedStrategy(){
  current_lethal_.reset(new pcl::PointCloud<pcl::PointXYZI>);
  return;
}

void PathBlockedStrategy::onInitialize()
{ 

  node_->declare_parameter(name_ + ".check_radius", rclcpp::ParameterValue(0.0));
  node_->get_parameter(name_ + ".check_radius", check_radius_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "check_radius: %.2f", check_radius_);//5.0
  prune_plan_blocked_ratio_ = 0.0;

}

void PathBlockedStrategy::updateLethalPointCloud(){
}

void PathBlockedStrategy::selfMark(){

  //@initial opinion as pass
  opinion_ = perception_3d::PASS;

  //@ there is no obstacle, so return ratio 0
  if(shared_data_->aggregate_observation_->points.size()<= 5 || shared_data_->pcl_prune_plan_.points.size()<=0){
    prune_plan_blocked_ratio_ = 0.0;
  }
  //@this method will return percent of point of pruneplan which conflict with obstacle  
  else{
    
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr pcl_perception_kdtree;
    pcl_perception_kdtree.reset(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    pcl_perception_kdtree->setInputCloud(shared_data_->aggregate_observation_);        
    
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    float prune_plan_orignal_size;
    float prune_plan_blocked_size;
    std::vector<geometry_msgs::msg::PoseStamped> prune_plan_point_blocked; //it could be deployed to a global variable
    for(int i=0;i<shared_data_->pcl_prune_plan_.points.size();i++){

      //@ intensity is the index of prune plan, negative is backward of robot,position is forward of robot
      if(shared_data_->pcl_prune_plan_.points[i].intensity<0)
        continue;

      if (pcl_perception_kdtree->radiusSearch (shared_data_->pcl_prune_plan_.points[i], check_radius_, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
        geometry_msgs::msg::PoseStamped temp_point;
        temp_point.pose.position.x = shared_data_->pcl_prune_plan_.points[i].x;
        temp_point.pose.position.y = shared_data_->pcl_prune_plan_.points[i].y;
        temp_point.pose.position.z = shared_data_->pcl_prune_plan_.points[i].z;
        prune_plan_point_blocked.push_back(temp_point);
      }
    }
    prune_plan_orignal_size = shared_data_->pcl_prune_plan_.points.size();
    prune_plan_blocked_size = prune_plan_point_blocked.size();
    prune_plan_blocked_ratio_ = (prune_plan_blocked_size) / (prune_plan_orignal_size)*100.0;
  }

  if(prune_plan_blocked_ratio_>0.0)
    opinion_ = perception_3d::PATH_BLOCKED_WAIT;
  
  //RCLCPP_WARN(node_->get_logger().get_child(name_), "%s: blocked_ratio:%.2f", name_.c_str(), prune_plan_blocked_ratio_ );
}

void PathBlockedStrategy::selfClear(){
  shared_data_->dgraph_update_request_[name_] = false;
}

void PathBlockedStrategy::resetdGraph(){}

double PathBlockedStrategy::get_dGraphValue(const unsigned int index){
  return 9999.0;
}

bool PathBlockedStrategy::isCurrent(){
  
  current_ = true;

  return current_;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PathBlockedStrategy::getObservation(){
  return sensor_current_observation_;
}
pcl::PointCloud<pcl::PointXYZI>::Ptr PathBlockedStrategy::getLethal(){
  return current_lethal_;
}
}//end of name space