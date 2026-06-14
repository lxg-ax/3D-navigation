// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <mpc_critics/collision_min_max_model.h>

PLUGINLIB_EXPORT_CLASS(mpc_critics::CollisionMinMaxModel, mpc_critics::ScoringModel)

namespace mpc_critics
{

CollisionMinMaxModel::CollisionMinMaxModel(){
  return;
  
}

void CollisionMinMaxModel::onInitialize(){

  node_->declare_parameter(name_ + ".weight", rclcpp::ParameterValue(1.0));
  node_->get_parameter(name_ + ".weight", weight_);
  RCLCPP_INFO(node_->get_logger().get_child(name_), "weight: %.2f", weight_);

}

double CollisionMinMaxModel::scoreTrajectory(base_trajectory::Trajectory &traj){
  
  if(shared_data_->pcl_perception_->points.size()<5){
    return 0.0;
  }
  


  for(unsigned int i=0;i<traj.getPointsSize();i++){

    base_trajectory::cuboid_min_max_t cmmt = traj.getCuboidMinMax(i);
    /*Assert for cuboids, we want user to have cuboids with the trajectory, otherwise it is dangerous*/
    assert( cmmt.first.x!=cmmt.second.x!=0 );
    
    pcl::PointXYZI pcl_traj_pose = traj.getPCLPoint(i);

    std::vector<int> id;
    std::vector<float> sqdist;
    //@The robot is not possible to be larger than 2 meters?
    shared_data_->pcl_perception_kdtree_->radiusSearch(pcl_traj_pose, 1.0, id, sqdist);
  
    for(auto pit=id.begin();pit!=id.end();pit++){
      auto pct_point = shared_data_->pcl_perception_->points[(*pit)];
      if(pct_point.x>=cmmt.first.x && pct_point.x<=cmmt.second.x &&
            pct_point.y>=cmmt.first.y && pct_point.y<=cmmt.second.y &&
              pct_point.z>=cmmt.first.z && pct_point.z<=cmmt.second.z)
      {
        return -1.0;
      }
      else{
        //@implement obstacle distance as rating?
        //traj.cost_ += obstacle_distance;
      }
    }
  }
  //@ there is obstacle, but the collision is passed, therefore return 0.0
  return 0.0;
}

}//end of name space