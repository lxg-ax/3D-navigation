// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#ifndef _DD_MPPI_THEORY_H__
#define _DD_MPPI_THEORY_H__

#include <random>

#include <trajectory_generators/trajectory_generator_theory.h>
#include <trajectory_generators/dd_simple_trajectory_generator_limits.h>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>

namespace trajectory_generators
{

// MPPI for differential drive. Reuses the existing critic chain as the
// running cost: we generate N perturbed control sequences around a warm-
// started nominal U, roll each one out, and let the planner score them
// via mpc_critics. combineByScores then computes softmax weights w_i =
// exp(-(S_i - min S)/lambda), averages the per-step controls into U*,
// and rolls U* out one more time as the reported best trajectory. The
// tail of U* is shifted forward to seed the next cycle.
//
// Differences from DDSimple:
//  * Sampling is in *control* space, not target velocity. Acc limits are
//    enforced per-step inside the rollout, not once at sample time.
//  * The argmin combiner is replaced by a softmax, so action-smoothness
//    survives even when several samples have similar costs.
//  * Each emitted Trajectory carries its full control sequence in
//    Trajectory.controls_, which is what combineByScores averages.
class MPPIDifferentialDriveTheory : public TrajectoryGeneratorTheory
{
public:
  MPPIDifferentialDriveTheory();

  bool hasMoreTrajectories() override;
  bool nextTrajectory(base_trajectory::Trajectory& traj) override;
  void initialise() override;

  bool combineByScores(std::vector<base_trajectory::Trajectory>& scored,
                       base_trajectory::Trajectory& combined) override;

protected:
  void onInitialize() override;

private:
  // Roll out a (v_t, omega_t) sequence from the current robot pose,
  // populating `traj` with body+global poses, cuboids, and `controls_`.
  // Returns false if any step violates kinematic limits or addPoint
  // fails. Acc clamping is applied here, not in the sampler, so that
  // softmax-averaged controls are also re-clamped on the final rollout.
  bool rolloutControls(const std::vector<Eigen::Vector2f>& U,
                       base_trajectory::Trajectory& traj);

  // Shift U* one step forward and pad the tail with a zero-omega coast,
  // matching standard MPPI warm-start. Called at the end of
  // combineByScores when a valid combined trajectory exists.
  void shiftNominal();

  // Limits + cuboid live in the same struct DDSimple uses, so we reuse
  // its yaml schema and the same vehicle envelope. controller_frequency
  // / sim_time / sim_granularity drive the horizon.
  std::shared_ptr<DDTrajectoryGeneratorLimits> limits_;
  pcl::PointCloud<pcl::PointXYZ> cuboid_;
  double controller_frequency_;
  double sim_time_;
  double sim_granularity_;
  double angular_sim_granularity_;

  // MPPI knobs. defaults are conservative; tune via overlay.
  int    num_samples_;        // N rollouts per cycle
  double lambda_;             // softmax temperature; smaller = sharper
  double sigma_v_;            // gaussian std on linear velocity
  double sigma_w_;            // gaussian std on angular velocity
  bool   reset_nominal_each_cycle_;
  double min_valid_fraction_; // if too many samples are rejected, skip combine

  // Discrete horizon: derived from sim_time / dt at initialise() time.
  int    horizon_;
  double dt_;

  // Warm-started nominal control sequence U_{t-1}, length horizon_.
  // Persists across cycles. Populated lazily in initialise().
  std::vector<Eigen::Vector2f> nominal_U_;

  // Per-cycle scratch: pre-sampled perturbed sequences. nextTrajectory
  // walks this in order. Repopulated at the start of every cycle inside
  // initialise().
  std::vector<std::vector<Eigen::Vector2f>> sampled_U_;
  unsigned int next_sample_index_;

  std::mt19937 rng_;
};

}  // namespace trajectory_generators

#endif
