// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#ifndef PERCEPTION_3D_PATH_BLOCKED_STRATEGY_H_
#define PERCEPTION_3D_PATH_BLOCKED_STRATEGY_H_

#include <perception_3d/sensor.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
/*Point cloud library*/
#include <pcl/point_types.h>

/*allows us to use pcl::transformPointCloud function*/
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl/common/transforms.h>


/*For distance calculation*/
#include <pcl/common/geometry.h>
#include <math.h>

/*Fast triangulation of unordered point clouds*/
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

/*RANSAC*/
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

namespace perception_3d
{

class PathBlockedStrategy: public Sensor{

  public:
    PathBlockedStrategy();

    virtual void onInitialize();
    virtual void selfClear();
    virtual void selfMark();
    virtual void updateLethalPointCloud();
    virtual pcl::PointCloud<pcl::PointXYZI>::Ptr getObservation();
    pcl::PointCloud<pcl::PointXYZI>::Ptr getLethal();
    virtual void resetdGraph();
    virtual double get_dGraphValue(const unsigned int index);
    virtual bool isCurrent();

  private:
    double prune_plan_blocked_ratio_;
    double check_radius_;
};

}//end of name space

#endif