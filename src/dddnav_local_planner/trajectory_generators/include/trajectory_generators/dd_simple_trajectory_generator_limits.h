// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#ifndef _DD_SIMPLE_TRAJECTORY_GENERATOR_LIMITS_H__
#define _DD_SIMPLE_TRAJECTORY_GENERATOR_LIMITS_H__

#include <Eigen/Core>

namespace trajectory_generators
{
class DDTrajectoryGeneratorLimits
{
public:

  double max_vel_x;
  double min_vel_x;
  double max_vel_theta;
  double min_vel_theta;
  double acc_lim_x;
  double acc_lim_theta;
  double deceleration_ratio;

  /*Motor constraint*/
  bool use_motor_constraint;
  double max_motor_shaft_rpm;
  double wheel_diameter;
  double gear_ratio;
  double robot_radius;

  /*For chopped plan, which are always required for local planner, therefore, move to limits*/
  double prune_forward;
  double prune_backward;

  DDTrajectoryGeneratorLimits() {}

  DDTrajectoryGeneratorLimits(
      double nmax_vel_x,
      double nmin_vel_x,
      double nmax_vel_theta,
      double nmin_vel_theta,
      double nacc_lim_x,
      double nacc_lim_theta,
      double nuse_motor_constraint,
      double nmax_motor_shaft_rpm,
      double nwheel_diameter,
      double ngear_ratio,
      double nrobot_radius,
      double nprune_forward,
      double nprune_backward,
      double ndeceleration_ratio):
        max_vel_x(nmax_vel_x),
        min_vel_x(nmin_vel_x),
        max_vel_theta(nmax_vel_theta),
        min_vel_theta(nmin_vel_theta),
        acc_lim_x(nacc_lim_x),
        acc_lim_theta(nacc_lim_theta),
        use_motor_constraint(nuse_motor_constraint),
        max_motor_shaft_rpm(nmax_motor_shaft_rpm),
        wheel_diameter(nwheel_diameter),
        gear_ratio(ngear_ratio),
        robot_radius(nrobot_radius),
        prune_forward(nprune_forward),
        prune_backward(nprune_backward),
        deceleration_ratio(ndeceleration_ratio)

{}

  ~DDTrajectoryGeneratorLimits() {}

  /**
   * @brief  Get the acceleration limits of the robot
   * @return  The acceleration limits of the robot
   */
  Eigen::Vector3f getAccLimits() {
    Eigen::Vector3f acc_limits;
    acc_limits[0] = acc_lim_x;
    acc_limits[2] = acc_lim_theta;
    return acc_limits;
  }

};

}
#endif // _DD_SIMPLE_TRAJECTORY_GENERATOR_LIMITS_H__
