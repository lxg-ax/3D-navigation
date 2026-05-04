/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _ONNI_SIMPLE_TRAJECTORY_GENERATOR_LIMITS_H__
#define _OMNI_SIMPLE_TRAJECTORY_GENERATOR_LIMITS_H__

#include <Eigen/Core>

namespace trajectory_generators
{
class OmniTrajectoryGeneratorLimits
{
public:

  double max_vel_x;
  double min_vel_x;
  double max_vel_y;
  double min_vel_y;
  double max_vel_trans;
  double min_vel_trans;
  double max_vel_theta;
  double min_vel_theta;
  double acc_lim_x;
  double acc_lim_y;
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

  OmniTrajectoryGeneratorLimits() {}

  OmniTrajectoryGeneratorLimits(
      double nmax_vel_x,
      double nmin_vel_x,
      double nmax_vel_y,
      double nmin_vel_y,
      double nmax_vel_trans,
      double nmin_vel_trans,
      double nmax_vel_theta,
      double nmin_vel_theta,
      double nacc_lim_x,
      double nacc_lim_y,
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
        max_vel_y(nmax_vel_y),
        min_vel_y(nmin_vel_y),
        max_vel_trans(nmax_vel_trans),
        min_vel_trans(nmin_vel_trans),
        max_vel_theta(nmax_vel_theta),
        min_vel_theta(nmin_vel_theta),
        acc_lim_x(nacc_lim_x),
        acc_lim_y(nacc_lim_y),
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

  ~OmniTrajectoryGeneratorLimits() {}

  /**
   * @brief  Get the acceleration limits of the robot
   * @return  The acceleration limits of the robot
   */
  Eigen::Vector3f getAccLimits() {
    Eigen::Vector3f acc_limits;
    acc_limits[0] = acc_lim_x;
    acc_limits[1] = acc_lim_y;
    acc_limits[2] = acc_lim_theta;
    return acc_limits;
  }

};

}
#endif // _OMNI_SIMPLE_TRAJECTORY_GENERATOR_LIMITS_H__
