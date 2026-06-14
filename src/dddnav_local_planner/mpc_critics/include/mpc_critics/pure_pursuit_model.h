// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <mpc_critics/scoring_model.h>

namespace mpc_critics
{

class PurePursuitModel: public ScoringModel{

  public:
    
    PurePursuitModel();
    virtual double scoreTrajectory(base_trajectory::Trajectory &traj);

  protected:

    virtual void onInitialize();

  private:
    double translation_weight_, orientation_weight_;
};

}//end of name space
