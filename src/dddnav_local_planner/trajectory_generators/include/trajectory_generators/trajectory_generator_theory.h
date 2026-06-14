// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

/*Debug*/
#include <chrono>

#include <pluginlib/class_list_macros.hpp>

/*path for trajectory*/
#include <base_trajectory/trajectory.h>
/*For tf2::matrix3x3 as quaternion to euler*/
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/common/transforms.h>

/*Data shared (if needed) between trajectory theories*/
#include <trajectory_generators/trajectory_shared_data.h>

namespace trajectory_generators
{

class TrajectoryGeneratorTheory{

  public:

    TrajectoryGeneratorTheory();

    void initialize(const std::string name, const rclcpp::Node::WeakPtr& weak_node);

    void setSharedData(std::shared_ptr<trajectory_generators::TrajectoryGeneratorSharedData> shared_data);

    virtual bool hasMoreTrajectories() = 0;
    virtual bool nextTrajectory(base_trajectory::Trajectory& _traj) = 0;
    //@ initialise is used for stacked generators to call every time to initialize the genertator
    virtual void initialise() = 0;

    /**
     * Combine the per-trajectory scores into a single best trajectory.
     *
     * Default implementation is argmin: pick the lowest-cost survivor (cost
     * < 0 means the critic chain rejected it, e.g. collision). Existing
     * generators (DDSimple, RotateInplace, OmniSimple) inherit this and
     * keep DWA-style behaviour with no change.
     *
     * MPPI overrides this to compute softmax weights w_i = exp(-(S_i -
     * min S)/lambda) over the scored candidates, average their stored
     * `controls_` sequences into a nominal U*, then roll U* out once with
     * the same kinematic model used for sampling. Without that override
     * the generator degenerates to denser DWA sampling and gives up the
     * action-smoothness MPPI is selected for.
     *
     * Returns true iff `combined` ends up with a valid trajectory (cost_
     * >= 0). `scored` may be reordered or read-only depending on the
     * implementation.
     */
    virtual bool combineByScores(
        std::vector<base_trajectory::Trajectory>& scored,
        base_trajectory::Trajectory& combined);

  protected:

    rclcpp::Node::SharedPtr node_;
    //@onInitialize is used to read ros param for the generator
    virtual void onInitialize() = 0;
    std::shared_ptr<trajectory_generators::TrajectoryGeneratorSharedData> shared_data_;
    std::string name_;


};


}//end of name space