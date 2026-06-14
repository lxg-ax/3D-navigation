// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <perception_3d/cluster_marking.h>


namespace perception_3d
{

Marking::~Marking(){
  //@ loop marking_ map to reset all ptr
  for(auto ix=marking_.begin(); ix!=marking_.end();ix++){
    for(auto iy=(*ix).second.begin(); iy!=(*ix).second.end();iy++){
      for(auto iz=(*iy).second.begin(); iz!=(*iy).second.end();iz++){
        (*iz).second.pc_.reset();
        (*iz).second.mc_.reset();
      } 
    }
  }
}

void Marking::computeMinDistanceFromObstacle2GroundNodes(  
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcptr, 
  const pcl::ModelCoefficients::Ptr& pcplaneptr,
  std::unordered_map<int, float>& nodes_of_min_distance){

  pcl::PointCloud<pcl::PointXYZI>::Ptr projected_cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
  pcl::ProjectInliers<pcl::PointXYZI> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (pcptr);
  proj.setModelCoefficients (pcplaneptr);
  proj.filter (*projected_cloud_cluster);

  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud (projected_cloud_cluster);
  sor.setLeafSize (0.1f, 0.1f, 0.1f);
  sor.filter (*projected_cloud_cluster);

  for(auto prj_pt_it=projected_cloud_cluster->points.begin();prj_pt_it!=projected_cloud_cluster->points.end();prj_pt_it++){
    pcl::PointXYZI pt;
    pt.x = (*prj_pt_it).x;
    pt.y = (*prj_pt_it).y;
    pt.z = (*prj_pt_it).z;

    std::vector<int> id_tmp;
    std::vector<float> sqdist_tmp;
    //@ We mark lethal
    if(shared_data_->kdtree_ground_->radiusSearch(pt, inflation_radius_, id_tmp, sqdist_tmp)){
      for(int i=0;i<id_tmp.size();i++){
        float dx = pt.x - shared_data_->pcl_ground_->points[id_tmp[i]].x;
        float dy = pt.y - shared_data_->pcl_ground_->points[id_tmp[i]].y;
        float dz = pt.z - shared_data_->pcl_ground_->points[id_tmp[i]].z;
        //@ Remove z value (see issue 8), we might have to review this assumption.
        float distance = sqrt(dx*dx + dy* dy);
        //RCLCPP_DEBUG(rclcpp::get_logger("cluster_marking"),"Distance xyz: %.2f, distance xy: %.2f", sqrt(sqdist_tmp[i]), distance);
        //if(nodes_of_min_distance.insert(std::make_pair(id_tmp[i], sqrt(sqdist_tmp[i]))).second == false)
        if(nodes_of_min_distance.insert(std::make_pair(id_tmp[i], distance)).second == false)
        {
          //@ key was presented
          //nodes_of_min_distance[id_tmp[i]] = std::min(nodes_of_min_distance[id_tmp[i]], sqrt(sqdist_tmp[i]));
          nodes_of_min_distance[id_tmp[i]] = std::min(nodes_of_min_distance[id_tmp[i]], distance);
        }

      }
      
    }
  }

}


void Marking::addPCPtr(const double cx, const double cy, const double cz, 
  const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcptr, 
  const pcl::ModelCoefficients::Ptr& pcplaneptr){
  
  int x = cx/xy_resolution_;
  int y = cy/xy_resolution_;
  int z = cz/height_resolution_;
  if(marking_[x][y][z].pc_!= nullptr){
    marking_[x][y][z].pc_.reset();
    marking_[x][y][z].mc_.reset();
  }
  marking_[x][y][z].pc_ = pcptr;
  marking_[x][y][z].mc_ = pcplaneptr;
  std::unordered_map<int, float> nodes_of_min_distance;
  computeMinDistanceFromObstacle2GroundNodes(pcptr, pcplaneptr, nodes_of_min_distance);
  marking_[x][y][z].nodes_of_min_distance_ = nodes_of_min_distance;
  
  for(auto id=nodes_of_min_distance.begin();id!=nodes_of_min_distance.end();id++){
    //RCLCPP_INFO(rclcpp::get_logger("ClusterMarking"), "%.2f", (*id).second);
    dGraph_->setValue((*id).first, (*id).second);
    if((*id).second<=inscribed_radius_)
      lethal_map_[(*id).first] = (*id).second;
  }
  
}

void Marking::removePCPtr(perception_3d::per_marking& per_marking){
  
  auto nodes_of_min_distance = per_marking.nodes_of_min_distance_;

  for(auto id=nodes_of_min_distance.begin();id!=nodes_of_min_distance.end();id++){
    dGraph_->clearValue((*id).first, 9999.0);
    if((*id).second<=inscribed_radius_)
      lethal_map_.erase((*id).first);
  }

  per_marking.pc_.reset();
  per_marking.mc_.reset();

}

void Marking::updateCleared(const std::vector<marking_voxel>& current_observation_ptr){
  /*
  for(auto it=current_observation_ptr.begin();it!=current_observation_ptr.end();it++){
    auto pcptr = marking_[(*it).x][(*it).y][(*it).z].first;
    auto pcplaneptr = marking_[(*it).x][(*it).y][(*it).z].second;

    //@Filter out empty cloud
    if(pcptr->points.empty())
      continue;

    std::unordered_map<int, float> nodes_of_min_distance;
    computeMinDistanceFromObstacle2GroundNodes(pcptr, pcplaneptr, nodes_of_min_distance);
    for(auto id=nodes_of_min_distance.begin();id!=nodes_of_min_distance.end();id++){
      dGraph_->setValue((*id).first,(*id).second);
    }
  }
  */
}

}//end of name space