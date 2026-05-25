// Copyright (c) 2024, DDDMobileRobot. BSD-3-Clause.
//
// pose_fusion_node — runs the SE(3) ESKF from eskf_se3.h.
//
// Inputs
//   ~/odom_topic        (default /Odometry, FAST-LIO 100 Hz)
//                       Used as the body-frame predictor.
//   ~/pose_topic        (default mcl_pose, MCL 3DL ~5 Hz)
//                       Used as the global-frame measurement.
//   ~/initial_pose_topic (default /initial_3d_pose, optional)
//                       Used to bootstrap the filter from an external nudge.
//
// Outputs
//   ~/output_odom_topic (default /odom_filtered) — fused 100 Hz odometry
//                       in map frame, child = base_link.
//   TF: map -> odom (computed from fused state and the latest FAST-LIO
//                    odom frame). Replaces MCL's TF — set publish_tf=false
//                    in mcl_3dl.yaml when this node is running.
//
// Usage notes
//   * The first MCL pose with reasonable covariance is used to initialise
//     the filter automatically. /initial_3d_pose is optional.
//   * If MCL drops out entirely the filter keeps integrating FAST-LIO; the
//     covariance grows, downstream consumers can tell things are getting
//     dicey via pose.covariance.
//   * If FAST-LIO drops out the filter freezes (no predict step). MCL
//     updates still come in but with no high-rate output.

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include "dddnav_pose_fusion/eskf_se3.h"

using std::placeholders::_1;

namespace dddnav_pose_fusion
{

class PoseFusionNode : public rclcpp::Node
{
public:
  PoseFusionNode()
  : Node("pose_fusion")
  {
    // ---- Topic / frame parameters ---------------------------------------
    declare_parameter<std::string>("odom_topic", "/Odometry");
    declare_parameter<std::string>("pose_topic", "mcl_pose");
    declare_parameter<std::string>("initial_pose_topic", "/initial_3d_pose");
    declare_parameter<std::string>("output_odom_topic", "/odom_filtered");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<bool>("publish_tf", true);

    // ---- Filter tuning ---------------------------------------------------
    // Process noise per second. Kept conservative because FAST-LIO odometry
    // is already quite clean; the tangent components are independent so a
    // diagonal works well in practice. Tune up if you see updates that are
    // too "stiff" (filter ignoring MCL corrections).
    declare_parameter<double>("proc_noise_pos", 0.05);   // m   / sqrt(s)
    declare_parameter<double>("proc_noise_rot", 0.02);   // rad / sqrt(s)

    // Measurement noise floor. If MCL publishes useful covariance we use
    // max(meas_floor, mcl_cov); otherwise we fall back to these constants.
    declare_parameter<double>("meas_noise_pos", 0.10);   // m
    declare_parameter<double>("meas_noise_rot", 0.05);   // rad

    declare_parameter<double>("init_cov_pos", 0.5);
    declare_parameter<double>("init_cov_rot", 0.3);

    // Mahalanobis gate (chi^2_6 quantile). 99% ≈ 16.81. Set <=0 to disable.
    declare_parameter<double>("mahalanobis_gate", 16.81);

    // Adaptive Mahalanobis gate. The fixed χ²₆ at the line above is the
    // baseline; we additionally inflate it whenever MCL itself reports a
    // high covariance (the filter is uncertain, so a wider innovation is
    // expected and should not be rejected). Final threshold is:
    //
    //   gate = mahalanobis_gate + adapt_gate_alpha * trace(P_mcl_xy)
    //
    // capped at adapt_gate_max. We also outright drop the MCL update if
    // its trace exceeds mcl_cov_reject_trace (m^2) — that almost always
    // means MCL hasn't converged and shouldn't pull the fused filter.
    // Set adapt_gate_alpha<=0 to disable adaptation; mcl_cov_reject_trace
    // <=0 disables the hard reject.
    declare_parameter<double>("adapt_gate_alpha",       30.0);
    declare_parameter<double>("adapt_gate_max",         60.0);
    declare_parameter<double>("mcl_cov_reject_trace",   3.0);   // m^2

    // ZUPT: when both translational + angular speed estimates are below the
    // configured thresholds we treat the body as stationary and shrink the
    // process noise. Helps avoid covariance ballooning while parked.
    declare_parameter<double>("zupt_lin_vel_thresh", 0.02);  // m/s
    declare_parameter<double>("zupt_ang_vel_thresh", 0.02);  // rad/s
    declare_parameter<double>("zupt_proc_scale",     0.1);   // multiplies Q

    // Adaptive Q: inflate process noise when the FAST-LIO front-end is having
    // trouble (residual rises). We subscribe to /fast_lio/health which packs
    // [res_mean (m), effct_feat_num] and scale Q by:
    //   q_scale_residual = 1 + adaptive_q_gain * max(0, residual - baseline) / baseline
    // capped at adaptive_q_max. Set adaptive_q_gain<=0 to disable.
    declare_parameter<std::string>("lio_health_topic",   "/fast_lio/health");
    declare_parameter<double>("adaptive_q_baseline",     0.05);  // metres
    declare_parameter<double>("adaptive_q_gain",         5.0);
    declare_parameter<double>("adaptive_q_max",          16.0);
    declare_parameter<int>("adaptive_q_min_feats",       50);    // <feats → boost
    // Gate failure handling: after N consecutive rejections, force-accept the
    // measurement (re-bootstrap if you prefer the term). Set to 0 to disable.
    declare_parameter<int>("gate_reset_after",       8);
    // Covariance bound: clamps each tangent diagonal so a long blackout does
    // not let the trace explode. Set to <= 0 to disable.
    declare_parameter<double>("max_cov_pos",         4.0);
    declare_parameter<double>("max_cov_rot",         1.0);
    // Auto-bootstrap: if no MCL pose arrives for this long after startup, use
    // (init_x/y/z) from the parameters as a self-init fallback.
    declare_parameter<double>("auto_init_timeout",   0.0);   // seconds, 0=off
    declare_parameter<double>("auto_init_x",         0.0);
    declare_parameter<double>("auto_init_y",         0.0);
    declare_parameter<double>("auto_init_z",         0.0);

    odom_topic_  = get_parameter("odom_topic").as_string();
    pose_topic_  = get_parameter("pose_topic").as_string();
    init_topic_  = get_parameter("initial_pose_topic").as_string();
    out_topic_   = get_parameter("output_odom_topic").as_string();
    map_frame_   = get_parameter("map_frame").as_string();
    odom_frame_  = get_parameter("odom_frame").as_string();
    base_frame_  = get_parameter("base_frame").as_string();
    publish_tf_  = get_parameter("publish_tf").as_bool();
    proc_pos_    = get_parameter("proc_noise_pos").as_double();
    proc_rot_    = get_parameter("proc_noise_rot").as_double();
    meas_pos_    = get_parameter("meas_noise_pos").as_double();
    meas_rot_    = get_parameter("meas_noise_rot").as_double();
    init_pos_    = get_parameter("init_cov_pos").as_double();
    init_rot_    = get_parameter("init_cov_rot").as_double();
    gate_chi2_   = get_parameter("mahalanobis_gate").as_double();
    adapt_gate_alpha_ = get_parameter("adapt_gate_alpha").as_double();
    adapt_gate_max_   = get_parameter("adapt_gate_max").as_double();
    mcl_cov_reject_trace_ = get_parameter("mcl_cov_reject_trace").as_double();
    zupt_v_      = get_parameter("zupt_lin_vel_thresh").as_double();
    zupt_w_      = get_parameter("zupt_ang_vel_thresh").as_double();
    zupt_scale_  = get_parameter("zupt_proc_scale").as_double();
    adaptive_q_baseline_ = get_parameter("adaptive_q_baseline").as_double();
    adaptive_q_gain_     = get_parameter("adaptive_q_gain").as_double();
    adaptive_q_max_      = get_parameter("adaptive_q_max").as_double();
    adaptive_q_min_feats_ = get_parameter("adaptive_q_min_feats").as_int();
    gate_reset_  = get_parameter("gate_reset_after").as_int();
    max_cov_pos_ = get_parameter("max_cov_pos").as_double();
    max_cov_rot_ = get_parameter("max_cov_rot").as_double();
    auto_init_to_ = get_parameter("auto_init_timeout").as_double();
    auto_init_x_  = get_parameter("auto_init_x").as_double();
    auto_init_y_  = get_parameter("auto_init_y").as_double();
    auto_init_z_  = get_parameter("auto_init_z").as_double();

    // ---- IO --------------------------------------------------------------
    rclcpp::QoS sensor_qos(rclcpp::KeepLast(50));
    sensor_qos.best_effort();

    sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, sensor_qos,
      std::bind(&PoseFusionNode::onFastLioOdom, this, _1));

    sub_pose_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      pose_topic_, 10,
      std::bind(&PoseFusionNode::onMclPose, this, _1));

    sub_init_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      init_topic_, 2,
      std::bind(&PoseFusionNode::onInitialPose, this, _1));

    // FAST-LIO health topic: drives adaptive Q.
    const auto health_topic =
      get_parameter("lio_health_topic").as_string();
    if (!health_topic.empty() && adaptive_q_gain_ > 0.0) {
      sub_health_ = create_subscription<std_msgs::msg::Float64MultiArray>(
        health_topic, 10,
        std::bind(&PoseFusionNode::onLioHealth, this, _1));
    }

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>(out_topic_, 50);
    if (publish_tf_) {
      tfb_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    }

    if (auto_init_to_ > 0.0) {
      auto_init_timer_ = create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(auto_init_to_ * 1000)),
        std::bind(&PoseFusionNode::tryAutoInit, this));
    }

    RCLCPP_INFO(get_logger(),
      "pose_fusion ready  predict<-%s  update<-%s  out=%s  publish_tf=%d",
      odom_topic_.c_str(), pose_topic_.c_str(), out_topic_.c_str(),
      static_cast<int>(publish_tf_));
  }

private:
  // -------------------------------------------------------------------------
  // FAST-LIO odometry: forms a body-frame delta from successive messages and
  // calls eskf.predict(). Output odometry is published immediately so we
  // inherit the upstream rate.
  // -------------------------------------------------------------------------
  void onFastLioOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);

    Eigen::Vector3d p_o_b(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);
    Eigen::Quaterniond q_o_b(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);
    q_o_b.normalize();

    const rclcpp::Time stamp(msg->header.stamp);

    if (have_last_odom_) {
      // body-frame delta: T_b1_b2 = T_o_b1^-1 * T_o_b2
      const Eigen::Quaterniond q_b1_o = q_o_b_last_.conjugate();
      const Eigen::Quaterniond delta_q = q_b1_o * q_o_b;
      const Eigen::Vector3d delta_p = q_b1_o * (p_o_b - p_o_b_last_);

      double dt = (stamp - stamp_last_).seconds();
      if (dt < 0) dt = 0;
      if (dt > 1.0) dt = 1.0;  // clamp on long pauses

      // ZUPT: when both translational and angular speeds are below the
      // configured thresholds we treat the body as stationary and shrink
      // the process noise. Scale stays >= zupt_scale_ to avoid singular Q.
      double q_scale = 1.0;
      if (dt > 1e-6) {
        const double v = delta_p.norm() / dt;
        const Eigen::Vector3d w_axis = logSO3Local(delta_q);
        const double w = w_axis.norm() / dt;
        if (v < zupt_v_ && w < zupt_w_) {
          q_scale = std::max(zupt_scale_, 1e-3);
        }
      }

      // Diagonal process-noise covariance, scaled by dt.
      EskfSE3::Cov Q = EskfSE3::Cov::Zero();
      const double q_total = q_scale * adaptive_q_scale_;
      const double qp = proc_pos_ * proc_pos_ * dt * q_total;
      const double qr = proc_rot_ * proc_rot_ * dt * q_total;
      Q(0, 0) = qp; Q(1, 1) = qp; Q(2, 2) = qp;
      Q(3, 3) = qr; Q(4, 4) = qr; Q(5, 5) = qr;

      if (eskf_.initialized()) {
        eskf_.predict(delta_p, delta_q, Q);
        clampCovariance();
      }
    }

    p_o_b_last_ = p_o_b;
    q_o_b_last_ = q_o_b;
    stamp_last_ = stamp;
    have_last_odom_ = true;

    if (eskf_.initialized()) {
      publishFused(stamp, p_o_b, q_o_b, msg->child_frame_id);
    }
  }

  // -------------------------------------------------------------------------
  // MCL 3DL global pose: measurement update.
  // -------------------------------------------------------------------------
  void onMclPose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);

    Eigen::Vector3d p_m_b(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);
    Eigen::Quaterniond q_m_b(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);
    q_m_b.normalize();

    if (!eskf_.initialized()) {
      EskfSE3::Cov P0 = EskfSE3::Cov::Zero();
      P0.diagonal() <<
        init_pos_ * init_pos_, init_pos_ * init_pos_, init_pos_ * init_pos_,
        init_rot_ * init_rot_, init_rot_ * init_rot_, init_rot_ * init_rot_;
      eskf_.initialize(p_m_b, q_m_b, P0);
      RCLCPP_INFO(get_logger(),
        "ESKF initialised from %s at (%.2f, %.2f, %.2f)",
        pose_topic_.c_str(), p_m_b.x(), p_m_b.y(), p_m_b.z());
      return;
    }

    // Use MCL-published covariance if it looks sane, otherwise fall back to
    // the configured floor. MCL 3DL packs cov as a 6x6 row-major (xyz, rpy).
    EskfSE3::Cov R = EskfSE3::Cov::Zero();
    bool used_msg_cov = false;
    double mcl_pos_trace = 0.0;
    if (msg->pose.covariance.size() == 36) {
      double trace = 0.0;
      for (int i = 0; i < 6; ++i) {
        trace += msg->pose.covariance[i * 6 + i];
      }
      if (trace > 0.0 && std::isfinite(trace)) {
        for (int i = 0; i < 6; ++i) {
          for (int j = 0; j < 6; ++j) {
            R(i, j) = msg->pose.covariance[i * 6 + j];
          }
        }
        // Floor each diagonal so we never trust the measurement more than
        // the configured limit (some MCL setups under-report).
        const double rp_floor = meas_pos_ * meas_pos_;
        const double rr_floor = meas_rot_ * meas_rot_;
        for (int i = 0; i < 3; ++i) R(i, i) = std::max(R(i, i), rp_floor);
        for (int i = 3; i < 6; ++i) R(i, i) = std::max(R(i, i), rr_floor);
        used_msg_cov = true;
        mcl_pos_trace = R(0, 0) + R(1, 1) + R(2, 2);
      }
    }
    if (!used_msg_cov) {
      const double rp = meas_pos_ * meas_pos_;
      const double rr = meas_rot_ * meas_rot_;
      R.diagonal() << rp, rp, rp, rr, rr, rr;
      mcl_pos_trace = 3.0 * rp;
    }

    // Hard reject MCL when its own position covariance is too large — the
    // particle cloud hasn't converged yet, and pulling the fused state to
    // such a noisy measurement is worse than coasting on FAST-LIO.
    if (mcl_cov_reject_trace_ > 0.0 &&
        mcl_pos_trace > mcl_cov_reject_trace_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "MCL pose dropped: pos cov trace %.3f > %.3f m^2",
        mcl_pos_trace, mcl_cov_reject_trace_);
      return;
    }

    // Adaptive gate: open it up when MCL itself is uncertain.
    double gate = gate_chi2_;
    if (gate_chi2_ > 0.0 && adapt_gate_alpha_ > 0.0) {
      gate = gate_chi2_ + adapt_gate_alpha_ * mcl_pos_trace;
      if (adapt_gate_max_ > 0.0) {
        gate = std::min(gate, adapt_gate_max_);
      }
    }

    const bool accepted = eskf_.update(p_m_b, q_m_b, R, gate);
    if (!accepted) {
      gate_reject_count_ += 1;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "MCL pose rejected by Mahalanobis gate (chi2 > %.2f, "
        "mcl_pos_trace=%.3f m^2), streak=%d",
        gate, mcl_pos_trace, gate_reject_count_);
      if (gate_reset_ > 0 && gate_reject_count_ >= gate_reset_) {
        EskfSE3::Cov P0 = EskfSE3::Cov::Zero();
        P0.diagonal() <<
          init_pos_ * init_pos_, init_pos_ * init_pos_, init_pos_ * init_pos_,
          init_rot_ * init_rot_, init_rot_ * init_rot_, init_rot_ * init_rot_;
        eskf_.initialize(p_m_b, q_m_b, P0);
        gate_reject_count_ = 0;
        RCLCPP_WARN(get_logger(),
          "ESKF re-bootstrapped after %d consecutive gate rejects "
          "at (%.2f, %.2f, %.2f)",
          gate_reset_, p_m_b.x(), p_m_b.y(), p_m_b.z());
      }
    } else {
      gate_reject_count_ = 0;
      clampCovariance();
    }
  }

  // -------------------------------------------------------------------------
  // Optional bootstrap from rviz initial pose.
  // -------------------------------------------------------------------------
  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    Eigen::Vector3d p(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);
    Eigen::Quaterniond q(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z);
    if (q.norm() < 1e-6) q = Eigen::Quaterniond::Identity();
    q.normalize();

    EskfSE3::Cov P0 = EskfSE3::Cov::Zero();
    P0.diagonal() <<
      init_pos_ * init_pos_, init_pos_ * init_pos_, init_pos_ * init_pos_,
      init_rot_ * init_rot_, init_rot_ * init_rot_, init_rot_ * init_rot_;
    eskf_.initialize(p, q, P0);
    RCLCPP_INFO(get_logger(),
      "ESKF re-initialised from %s at (%.2f, %.2f, %.2f)",
      init_topic_.c_str(), p.x(), p.y(), p.z());
  }

  // -------------------------------------------------------------------------
  // Publish fused odometry + map->odom TF.
  //   Fused state is T_m_b (body in map). FAST-LIO gives us T_o_b (body in
  //   odom). map->odom = T_m_b * T_o_b^-1.
  // -------------------------------------------------------------------------
  void publishFused(
    const rclcpp::Time & stamp,
    const Eigen::Vector3d & p_o_b,
    const Eigen::Quaterniond & q_o_b,
    const std::string & child_frame_in)
  {
    const Eigen::Vector3d & p_m_b = eskf_.position();
    const Eigen::Quaterniond & q_m_b = eskf_.orientation();

    // ---- /odom_filtered (map -> base) -----------------------------------
    nav_msgs::msg::Odometry out;
    out.header.stamp = stamp;
    out.header.frame_id = map_frame_;
    out.child_frame_id = child_frame_in.empty() ? base_frame_ : child_frame_in;
    out.pose.pose.position.x = p_m_b.x();
    out.pose.pose.position.y = p_m_b.y();
    out.pose.pose.position.z = p_m_b.z();
    out.pose.pose.orientation.w = q_m_b.w();
    out.pose.pose.orientation.x = q_m_b.x();
    out.pose.pose.orientation.y = q_m_b.y();
    out.pose.pose.orientation.z = q_m_b.z();
    const auto & P = eskf_.covariance();
    for (int i = 0; i < 6; ++i) {
      for (int j = 0; j < 6; ++j) {
        out.pose.covariance[i * 6 + j] = P(i, j);
      }
    }
    pub_odom_->publish(out);

    // ---- TF map -> odom -------------------------------------------------
    if (publish_tf_ && tfb_) {
      // T_m_o = T_m_b * T_b_o = T_m_b * T_o_b^-1
      const Eigen::Quaterniond q_o_b_inv = q_o_b.conjugate();
      const Eigen::Quaterniond q_m_o = (q_m_b * q_o_b_inv).normalized();
      const Eigen::Vector3d p_m_o = p_m_b - (q_m_o * p_o_b);

      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = map_frame_;
      tf.child_frame_id = odom_frame_;
      tf.transform.translation.x = p_m_o.x();
      tf.transform.translation.y = p_m_o.y();
      tf.transform.translation.z = p_m_o.z();
      tf.transform.rotation.w = q_m_o.w();
      tf.transform.rotation.x = q_m_o.x();
      tf.transform.rotation.y = q_m_o.y();
      tf.transform.rotation.z = q_m_o.z();
      tfb_->sendTransform(tf);
    }
  }

  // -------------------------------------------------------------------------
  // FAST-LIO residual / feature-count health update. Anything above the
  // configured baseline ramps up adaptive_q_scale_ which the predict step
  // multiplies into Q, so the filter trusts MCL more during jolts / dynamic
  // scenes / featureless corridors.
  // -------------------------------------------------------------------------
  void onLioHealth(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
  {
    if (msg->data.size() < 1 || adaptive_q_gain_ <= 0.0) return;
    const double residual = msg->data[0];
    const int feats = (msg->data.size() >= 2) ?
                      static_cast<int>(msg->data[1]) : 1000;

    double scale = 1.0;
    if (residual > adaptive_q_baseline_ && adaptive_q_baseline_ > 1e-9) {
      const double over = (residual - adaptive_q_baseline_) /
                          adaptive_q_baseline_;
      scale = 1.0 + adaptive_q_gain_ * over;
    }
    if (feats > 0 && feats < adaptive_q_min_feats_) {
      // Sparse correspondence -> front-end is starving, boost MCL weight too.
      scale = std::max(scale, 4.0);
    }
    scale = std::min(scale, adaptive_q_max_);
    std::lock_guard<std::mutex> lk(mtx_);
    adaptive_q_scale_ = scale;
  }

  // ---- helpers ---------------------------------------------------------
  // Local copy of the SO(3) log so we don't have to expose it from the
  // EskfSE3 header. Returns axis-angle vector (rad).
  static Eigen::Vector3d logSO3Local(const Eigen::Quaterniond & q_in)
  {
    Eigen::Quaterniond q = q_in.normalized();
    if (q.w() < 0) q.coeffs() = -q.coeffs();
    const Eigen::Vector3d v(q.x(), q.y(), q.z());
    const double n = v.norm();
    if (n < 1e-9) return 2.0 * v;
    return v * (2.0 * std::atan2(n, q.w()) / n);
  }

  void clampCovariance()
  {
    if (max_cov_pos_ <= 0.0 && max_cov_rot_ <= 0.0) return;
    EskfSE3::Cov P = eskf_.covariance();
    bool changed = false;
    for (int i = 0; i < 3; ++i) {
      if (max_cov_pos_ > 0.0 && P(i, i) > max_cov_pos_) {
        P(i, i) = max_cov_pos_; changed = true;
      }
    }
    for (int i = 3; i < 6; ++i) {
      if (max_cov_rot_ > 0.0 && P(i, i) > max_cov_rot_) {
        P(i, i) = max_cov_rot_; changed = true;
      }
    }
    if (changed) {
      eskf_.initialize(eskf_.position(), eskf_.orientation(), P);
    }
  }

  // Auto-init fallback: if no MCL pose has arrived after auto_init_to_
  // seconds we drop the filter at the configured initial pose so downstream
  // consumers can at least see /odom_filtered. MCL takes over once it
  // converges (its first message will reset the filter via the gate logic).
  void tryAutoInit()
  {
    std::lock_guard<std::mutex> lk(mtx_);
    auto_init_timer_->cancel();
    if (eskf_.initialized()) return;
    Eigen::Vector3d p(auto_init_x_, auto_init_y_, auto_init_z_);
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    EskfSE3::Cov P0 = EskfSE3::Cov::Zero();
    const double pos2 = init_pos_ * init_pos_;
    const double rot2 = init_rot_ * init_rot_;
    P0.diagonal() << pos2, pos2, pos2, rot2, rot2, rot2;
    eskf_.initialize(p, q, P0);
    RCLCPP_WARN(get_logger(),
      "ESKF auto-initialised after %.1fs without MCL at (%.2f, %.2f, %.2f)",
      auto_init_to_, p.x(), p.y(), p.z());
  }

  // ---- State -----------------------------------------------------------
  std::mutex mtx_;
  EskfSE3 eskf_;

  bool have_last_odom_{false};
  Eigen::Vector3d p_o_b_last_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q_o_b_last_{Eigen::Quaterniond::Identity()};
  rclcpp::Time stamp_last_;

  // ---- IO --------------------------------------------------------------
  std::string odom_topic_, pose_topic_, init_topic_, out_topic_;
  std::string map_frame_, odom_frame_, base_frame_;
  bool publish_tf_;
  double proc_pos_, proc_rot_, meas_pos_, meas_rot_;
  double init_pos_, init_rot_, gate_chi2_;
  double adapt_gate_alpha_{0.0};
  double adapt_gate_max_{0.0};
  double mcl_cov_reject_trace_{0.0};
  double zupt_v_{0.02}, zupt_w_{0.02}, zupt_scale_{0.1};
  double adaptive_q_baseline_{0.05};
  double adaptive_q_gain_{0.0};
  double adaptive_q_max_{16.0};
  int    adaptive_q_min_feats_{50};
  double adaptive_q_scale_{1.0};
  int gate_reset_{0};
  int gate_reject_count_{0};
  double max_cov_pos_{0.0}, max_cov_rot_{0.0};
  double auto_init_to_{0.0};
  double auto_init_x_{0.0}, auto_init_y_{0.0}, auto_init_z_{0.0};
  rclcpp::TimerBase::SharedPtr auto_init_timer_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_pose_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_init_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_health_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfb_;
};

}  // namespace dddnav_pose_fusion

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dddnav_pose_fusion::PoseFusionNode>());
  rclcpp::shutdown();
  return 0;
}
