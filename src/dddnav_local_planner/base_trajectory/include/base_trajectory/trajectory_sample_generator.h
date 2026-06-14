// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#ifndef TRAJECTORY_SAMPLE_GENERATOR_H_
#define TRAJECTORY_SAMPLE_GENERATOR_H_

/*path for trajectory*/
#include <base_trajectory/trajectory.h>

namespace base_trajectory {

/**
 * @class TrajectorySampleGenerator
 * @brief Provides an interface for navigation trajectory generators
 */
class TrajectorySampleGenerator {
public:

  //virtual void readParameters(std::string name) = 0;

  /**
   * Whether this generator can create more trajectories
   */
  virtual bool hasMoreTrajectories() = 0;

  /**
   * Iterate to the next trajectory
   */
  virtual bool nextTrajectory(base_trajectory::Trajectory& comp_traj) = 0;

  /**
   * @brief  Virtual destructor for the interface
   */
  virtual ~TrajectorySampleGenerator() {}

protected:
  TrajectorySampleGenerator() {}

};

} // end namespace

#endif /* TRAJECTORY_SAMPLE_GENERATOR_H_ */
