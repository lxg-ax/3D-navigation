// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <trajectory_generators/trajectory_generator_theory.h>
#include <trajectory_generators/dd_simple_trajectory_generator_limits.h>
#include <trajectory_generators/dd_simple_trajectory_generator_params.h>
#include <trajectory_generators/velocity_iterator.h>

/*getMinMax3D*/
#include <pcl/common/common.h>

namespace trajectory_generators
{

class DDRotateInplaceTheory: public TrajectoryGeneratorTheory{

  public:
    
    DDRotateInplaceTheory();

    virtual bool hasMoreTrajectories();
    virtual bool nextTrajectory(base_trajectory::Trajectory& _traj);

  private:
    void initialise();
    bool isMotorConstraintSatisfied(Eigen::Vector3f& vel_samp);

    bool generateTrajectory(
        Eigen::Vector3f sample_target_vel,
        base_trajectory::Trajectory& traj);

    Eigen::Vector3f computeNewPositions(const Eigen::Vector3f& pos,
        const Eigen::Vector3f& vel, double dt);

    double rotation_speed_;
    
  protected:

    virtual void onInitialize();

    std::shared_ptr<trajectory_generators::DDTrajectoryGeneratorLimits> limits_;
    std::shared_ptr<trajectory_generators::DDTrajectoryGeneratorParams> params_;

    unsigned int next_sample_index_;
    // to store sample params of each sample between init and generation
    std::vector<Eigen::Vector3f> sample_params_;
};

}//end of name space
