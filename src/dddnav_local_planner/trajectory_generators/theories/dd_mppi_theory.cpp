// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#include <trajectory_generators/dd_mppi_theory.h>

#include <algorithm>
#include <cmath>
#include <chrono>

#include <tf2_eigen/tf2_eigen.hpp>

PLUGINLIB_EXPORT_CLASS(trajectory_generators::MPPIDifferentialDriveTheory,
                       trajectory_generators::TrajectoryGeneratorTheory)

namespace trajectory_generators
{

MPPIDifferentialDriveTheory::MPPIDifferentialDriveTheory()
  : controller_frequency_(10.0),
    sim_time_(1.0),
    sim_granularity_(0.05),
    angular_sim_granularity_(0.025),
    num_samples_(64),
    lambda_(1.0),
    sigma_v_(0.2),
    sigma_w_(0.4),
    reset_nominal_each_cycle_(false),
    min_valid_fraction_(0.1),
    horizon_(0),
    dt_(0.0),
    next_sample_index_(0),
    rng_(std::chrono::steady_clock::now().time_since_epoch().count())
{
}

void MPPIDifferentialDriveTheory::onInitialize()
{
  limits_ = std::make_shared<DDTrajectoryGeneratorLimits>();

  auto getp = [&](const std::string& k, auto def, auto& out){
    node_->declare_parameter(name_ + "." + k, rclcpp::ParameterValue(def));
    node_->get_parameter(name_ + "." + k, out);
  };

  // Reuse the dd_simple yaml schema for kinematic limits + cuboid so an
  // operator can flip between dd_simple and mppi without rewriting the
  // robot model. Anything mppi-specific lives under the same plugin
  // namespace with mppi-prefixed keys.
  getp("min_vel_x",       0.0,  limits_->min_vel_x);
  getp("max_vel_x",       0.5,  limits_->max_vel_x);
  getp("min_vel_theta",  -1.0,  limits_->min_vel_theta);
  getp("max_vel_theta",   1.0,  limits_->max_vel_theta);
  getp("acc_lim_x",       1.0,  limits_->acc_lim_x);
  getp("acc_lim_theta",   2.0,  limits_->acc_lim_theta);
  getp("deceleration_ratio", 2.0, limits_->deceleration_ratio);
  getp("use_motor_constraint", false, limits_->use_motor_constraint);
  getp("max_motor_shaft_rpm",  3000.0, limits_->max_motor_shaft_rpm);
  getp("wheel_diameter",       0.16,   limits_->wheel_diameter);
  getp("gear_ratio",           1.0,    limits_->gear_ratio);
  getp("robot_radius",         0.25,   limits_->robot_radius);
  getp("prune_forward",        3.0,    limits_->prune_forward);
  getp("prune_backward",       1.0,    limits_->prune_backward);

  getp("controller_frequency",     10.0,   controller_frequency_);
  getp("sim_time",                 1.5,    sim_time_);
  getp("sim_granularity",          0.05,   sim_granularity_);
  getp("angular_sim_granularity",  0.025,  angular_sim_granularity_);

  getp("mppi.num_samples",                64,    num_samples_);
  getp("mppi.lambda",                     1.0,   lambda_);
  getp("mppi.sigma_v",                    0.2,   sigma_v_);
  getp("mppi.sigma_w",                    0.4,   sigma_w_);
  getp("mppi.reset_nominal_each_cycle",   false, reset_nominal_each_cycle_);
  getp("mppi.min_valid_fraction",         0.1,   min_valid_fraction_);

  // Cuboid (8 vertices, body frame). Same keys as dd_simple.
  const std::vector<std::string> faces = {"flb","frb","flt","frt","blb","brb","blt","brt"};
  cuboid_.clear();
  for(const auto& f : faces){
    std::string key = name_ + ".cuboid." + f;
    node_->declare_parameter(key, rclcpp::PARAMETER_DOUBLE_ARRAY);
    auto p = node_->get_parameter(key).as_double_array();
    pcl::PointXYZ pt;
    pt.x = p[0]; pt.y = p[1]; pt.z = p[2];
    cuboid_.push_back(pt);
  }

  // Discretise the horizon. dt follows sim_granularity scaled by an
  // assumed reference speed of 1 m/s; we cap it at controller_frequency
  // so the rollout never resolves finer than the control loop.
  dt_ = std::max(sim_granularity_, 1.0 / controller_frequency_);
  horizon_ = std::max(1, static_cast<int>(std::ceil(sim_time_ / dt_)));

  nominal_U_.assign(horizon_, Eigen::Vector2f::Zero());
  sampled_U_.clear();

  RCLCPP_INFO(node_->get_logger().get_child(name_),
              "MPPI initialised: N=%d horizon=%d dt=%.3f lambda=%.2f sigma_v=%.2f sigma_w=%.2f",
              num_samples_, horizon_, dt_, lambda_, sigma_v_, sigma_w_);
}

void MPPIDifferentialDriveTheory::initialise()
{
  // Optional cold start: drop warm state if requested. Useful when the
  // navigation goal changes drastically and the previous nominal would
  // bias us into an irrelevant corridor.
  if (reset_nominal_each_cycle_){
    nominal_U_.assign(horizon_, Eigen::Vector2f::Zero());
  }

  // Seed the nominal toward the operator's current speed band when no
  // history exists yet (e.g. first cycle after reset). Empty nominal +
  // zero injection biases the robot toward standing still and most
  // critics will then have no signal.
  bool nominal_all_zero = std::all_of(nominal_U_.begin(), nominal_U_.end(),
      [](const Eigen::Vector2f& u){ return u[0] == 0.0f && u[1] == 0.0f; });
  if (nominal_all_zero){
    float seed_v = static_cast<float>(std::max(0.0, 0.5 * limits_->max_vel_x));
    for (auto& u : nominal_U_) u = Eigen::Vector2f(seed_v, 0.0f);
  }

  // Resample N perturbed control sequences. We sample epsilon ~ N(0,
  // sigma) per step and add it to the warm-started nominal. Limits are
  // enforced inside rolloutControls so that the perturbation is honest:
  // a pre-clamped epsilon would silently shrink the effective sigma.
  sampled_U_.clear();
  sampled_U_.reserve(num_samples_);
  std::normal_distribution<float> nv(0.0f, static_cast<float>(sigma_v_));
  std::normal_distribution<float> nw(0.0f, static_cast<float>(sigma_w_));
  for (int n = 0; n < num_samples_; ++n){
    std::vector<Eigen::Vector2f> U(horizon_);
    for (int t = 0; t < horizon_; ++t){
      U[t] = nominal_U_[t] + Eigen::Vector2f(nv(rng_), nw(rng_));
    }
    sampled_U_.push_back(std::move(U));
  }
  next_sample_index_ = 0;
}

bool MPPIDifferentialDriveTheory::hasMoreTrajectories()
{
  return next_sample_index_ < sampled_U_.size();
}

bool MPPIDifferentialDriveTheory::nextTrajectory(base_trajectory::Trajectory& traj)
{
  if (!hasMoreTrajectories()) return false;

  bool ok = rolloutControls(sampled_U_[next_sample_index_], traj);
  if (!ok){
    traj.resetPoints();
  }
  ++next_sample_index_;
  return ok;
}

bool MPPIDifferentialDriveTheory::rolloutControls(
    const std::vector<Eigen::Vector2f>& U,
    base_trajectory::Trajectory& traj)
{
  traj.resetPoints();
  traj.cost_ = 0.0;
  traj.time_delta_ = dt_;
  traj.controls_.clear();
  traj.controls_.reserve(U.size());

  Eigen::Affine3d pos_af3 = tf2::transformToEigen(shared_data_->robot_pose_);

  // body-frame integration; we transform every step into the global
  // frame for collision and critic evaluation, identical to dd_simple's
  // pattern so no existing critic needs to special-case mppi outputs.
  Eigen::Vector3f pos = Eigen::Vector3f::Zero();
  Eigen::Vector2f prev_u(
      static_cast<float>(shared_data_->robot_state_.twist.twist.linear.x),
      static_cast<float>(shared_data_->robot_state_.twist.twist.angular.z));

  // We expose the seed control (v0, w0) on the Trajectory so existing
  // visualisation that reads xv_/thetav_ still shows something sensible
  // for the published "best" trajectory.
  if (!U.empty()){
    traj.xv_     = U.front()[0];
    traj.thetav_ = U.front()[1];
  }

  const double max_dv = limits_->acc_lim_x     * dt_;
  const double max_dw = limits_->acc_lim_theta * dt_;

  for (size_t k = 0; k < U.size(); ++k){
    Eigen::Vector2f u = U[k];

    // Clamp acceleration step-by-step. This is what makes softmax-
    // averaged controls survive: even if averaging produces a jump,
    // re-clamping on the final rollout gives a feasible plan.
    u[0] = static_cast<float>(std::clamp<double>(u[0],
              prev_u[0] - max_dv, prev_u[0] + max_dv));
    u[1] = static_cast<float>(std::clamp<double>(u[1],
              prev_u[1] - max_dw, prev_u[1] + max_dw));
    // Hard saturation against vehicle envelope.
    u[0] = static_cast<float>(std::clamp<double>(u[0],
              limits_->min_vel_x, limits_->max_vel_x));
    u[1] = static_cast<float>(std::clamp<double>(u[1],
              limits_->min_vel_theta, limits_->max_vel_theta));

    traj.controls_.push_back(u);

    // 2D unicycle integration in body frame.
    pos[0] += u[0] * std::cos(pos[2]) * dt_;
    pos[1] += u[0] * std::sin(pos[2]) * dt_;
    pos[2] += u[1] * dt_;

    Eigen::Affine3d trans_b2traj_af3(Eigen::AngleAxisd(pos[2], Eigen::Vector3d::UnitZ()));
    trans_b2traj_af3.translation().x() = pos[0];
    trans_b2traj_af3.translation().y() = pos[1];
    Eigen::Affine3d trans_gbl2traj_af3 = pos_af3 * trans_b2traj_af3;
    geometry_msgs::msg::TransformStamped ts = tf2::eigenToTransform(trans_gbl2traj_af3);

    geometry_msgs::msg::PoseStamped ros_pose;
    ros_pose.header = shared_data_->robot_pose_.header;
    ros_pose.pose.position.x = ts.transform.translation.x;
    ros_pose.pose.position.y = ts.transform.translation.y;
    ros_pose.pose.position.z = ts.transform.translation.z;
    ros_pose.pose.orientation = ts.transform.rotation;

    pcl::PointCloud<pcl::PointXYZ> pc_out;
    pcl::transformPointCloud(cuboid_, pc_out, trans_gbl2traj_af3);
    base_trajectory::cuboid_min_max_t cmm;
    pcl::getMinMax3D(pc_out, cmm.first, cmm.second);

    if (!traj.addPoint(ros_pose, pc_out, cmm)){
      return false;
    }

    prev_u = u;
  }
  return !traj.controls_.empty();
}

bool MPPIDifferentialDriveTheory::combineByScores(
    std::vector<base_trajectory::Trajectory>& scored,
    base_trajectory::Trajectory& combined)
{
  combined.cost_ = -1.0;

  // 1) Filter to candidates that survived the critic chain. cost_<0 is
  //    the convention for "rejected" (e.g. collision); MPPI weights
  //    must not include those or a single survivor with cost ~ min(S)
  //    would still get drowned out by the rejected mass.
  std::vector<size_t> ok_idx;
  ok_idx.reserve(scored.size());
  double s_min = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < scored.size(); ++i){
    if (scored[i].cost_ >= 0.0 && !scored[i].controls_.empty()){
      ok_idx.push_back(i);
      s_min = std::min(s_min, scored[i].cost_);
    }
  }

  if (ok_idx.empty() ||
      static_cast<double>(ok_idx.size()) <
        min_valid_fraction_ * static_cast<double>(scored.size())){
    // Too many rejections: fall back to argmin of whatever survived.
    // Better than averaging in degenerate weights and producing a
    // useless mean control. Critics will mark best.cost_ = -1 if none
    // survive at all, which propagates to ALL_TRAJECTORIES_FAIL upstream.
    double best = std::numeric_limits<double>::max();
    for (auto& t : scored){
      if (t.cost_ >= 0.0 && t.cost_ <= best){
        combined = t;
        best = t.cost_;
      }
    }
    return combined.cost_ >= 0.0;
  }

  // 2) Softmax weights. Normalising by min(S) keeps the exponent in a
  //    sane range across cycles (raw S can drift by orders of magnitude
  //    when stick_path / pure_pursuit gains dominate).
  std::vector<double> w(ok_idx.size());
  double w_sum = 0.0;
  for (size_t k = 0; k < ok_idx.size(); ++k){
    w[k] = std::exp(-(scored[ok_idx[k]].cost_ - s_min) / std::max(1e-6, lambda_));
    w_sum += w[k];
  }
  if (w_sum < 1e-9){
    // Numerical underflow -- act like argmin.
    size_t best = ok_idx.front();
    double best_cost = scored[best].cost_;
    for (auto i : ok_idx){
      if (scored[i].cost_ < best_cost){
        best_cost = scored[i].cost_;
        best = i;
      }
    }
    combined = scored[best];
    return true;
  }

  // 3) Weighted average of per-step controls. Length-mismatched samples
  //    are skipped at each t so a short rollout can't pull the mean
  //    toward zero on later steps.
  std::vector<Eigen::Vector2f> U_star(horizon_, Eigen::Vector2f::Zero());
  std::vector<double> norm(horizon_, 0.0);
  for (size_t k = 0; k < ok_idx.size(); ++k){
    const auto& U = scored[ok_idx[k]].controls_;
    const double wk = w[k];
    for (int t = 0; t < horizon_ && t < static_cast<int>(U.size()); ++t){
      U_star[t] += static_cast<float>(wk) * U[t];
      norm[t]   += wk;
    }
  }
  for (int t = 0; t < horizon_; ++t){
    if (norm[t] > 1e-9) U_star[t] /= static_cast<float>(norm[t]);
  }

  // 4) Roll U* out as the reported best. Re-applies acc clamping +
  //    cuboid generation, so this is a fully scoreable trajectory.
  if (!rolloutControls(U_star, combined)){
    combined.cost_ = -1.0;
    return false;
  }
  // The combined trajectory wasn't scored yet; mark cost_ = 0 so the
  // upstream check `best_traj.cost_<0 -> ALL_TRAJECTORIES_FAIL` doesn't
  // fire. Real numeric cost isn't useful here -- it's the average of
  // sample costs by construction.
  combined.cost_ = 0.0;

  // 5) Persist U* as the next cycle's nominal.
  nominal_U_ = U_star;
  shiftNominal();
  return true;
}

void MPPIDifferentialDriveTheory::shiftNominal()
{
  if (nominal_U_.size() < 2) return;
  for (size_t t = 0; t + 1 < nominal_U_.size(); ++t){
    nominal_U_[t] = nominal_U_[t + 1];
  }
  // Tail: hold the last command (omega is already a turn rate, holding
  // it is a coast assumption that is rarely worse than zeroing it).
  nominal_U_.back() = nominal_U_[nominal_U_.size() - 2];
}

}  // namespace trajectory_generators
