// Copyright (c) 2016-2017, the mcl_3dl authors
// SPDX-License-Identifier: BSD-3-Clause

#ifndef MCL_3DL_UTILITIES_H
#define MCL_3DL_UTILITIES_H

#include "rclcpp/rclcpp.hpp"
#include <map>
#include <vector>
#include <string>
#include <memory>
#include <cmath>
#include <algorithm>
#include <utility>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>

/*allows us to use pcl::transformPointCloud function*/
#include <tf2_eigen/tf2_eigen.hpp>

/*For pcl::transformPointCloud, dont use #include <pcl/common/transforms.h> ???*/
#include <pcl/common/transforms.h>

//@kdtree and normals
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time)
)

typedef PointXYZIRPYT PointTypePose;

namespace mcl_3dl
{
  typedef pcl::PointXYZI pcl_t;
}

#endif  // MCL_3DL_UTILITIES_H
