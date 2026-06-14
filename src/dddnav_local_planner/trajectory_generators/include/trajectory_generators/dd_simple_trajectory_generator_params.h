// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#ifndef _DD_SIMPLE_TRAJECTORY_GENERATOR_PARAMS_H__
#define _DD_SIMPLE_TRAJECTORY_GENERATOR_PARAMS_H__

#include <Eigen/Core>
//@ pcl for cuboid
#include <pcl/point_cloud.h>

namespace trajectory_generators
{
class DDTrajectoryGeneratorParams
{
public:

  double controller_frequency;
  double sim_time;
  double linear_x_sample;
  double angular_z_sample;
  double sim_granularity;
  double angular_sim_granularity;
  pcl::PointCloud<pcl::PointXYZ> cuboid;

  DDTrajectoryGeneratorParams() {}

  DDTrajectoryGeneratorParams(
      double ncontroller_frequency,
      double nsim_time,
      double nlinear_x_sample,
      double nangular_z_sample,
      double nsim_granularity,
      double nangular_sim_granularity):
        controller_frequency(ncontroller_frequency),
        sim_time(nsim_time),
        linear_x_sample(nlinear_x_sample),
        angular_z_sample(nangular_z_sample),
        sim_granularity(nsim_granularity),
        angular_sim_granularity(nangular_sim_granularity)

{}

  ~DDTrajectoryGeneratorParams() {}


};

}
#endif // _DD_SIMPLE_TRAJECTORY_GENERATOR_PARAMS_H__
