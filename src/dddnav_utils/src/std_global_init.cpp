// Copyright (c) 2024, DDDMobileRobot. BSD-3-Clause.
//
// std_global_init: STD (Stable Triangle Descriptor) based global localisation
// bootstrap + in-place re-localisation watchdog.
//
// Replaces the earlier Scan Context implementation. Two key differences from
// the SC version:
//
//   1. STD's SearchLoop returns a 6-DoF relative transform (query → candidate
//      keyframe), not just a yaw shift. We compose it with the keyframe's
//      world pose to get the seed instead of copying roll/pitch verbatim.
//      Roll/pitch from the relative pose stays small for ground robots
//      (Mid360 near-level), so MCL still converges.
//
//   2. The match metric is plane-ICP score (higher = better, usually 0..1)
//      rather than cosine distance (lower = better). All thresholds flip
//      sign / direction.
//
// Two modes share the same STD database that LIO-SAM dumps at mapping time
// (`<map_dir>/lio_sam/std_db.bin`) plus the keyframe poses
// (`<map_dir>/poses.pcd`):
//
//   1. Bootstrap (no MCL fix yet): on every LiDAR frame we build STDs from
//      the body-frame cloud, query the DB, require N consecutive frames to
//      agree on the same candidate before publishing /initial_3d_pose.
//
//   2. Watchdog (MCL has been fixing the pose for a while): the same query
//      runs at low rate. We accept a re-localisation only when the STD
//      candidate is (a) strong (score above `relocate_min_score`), (b)
//      consistent across `relocate_consensus` ticks, and (c) clearly NOT
//      near the keyframe MCL currently sits on (so we don't re-trigger when
//      MCL is already correct), but is plausibly close to the odom prior
//      (so we don't flip across the map under aliasing).
//
// Notes
//   * Every published initial pose carries an inflated covariance (~1m
//     position, ~0.3rad yaw) so MCL spreads particles around the seed
//     before locking in.
//   * The watchdog is OFF when `enable_watchdog=false`. Bootstrap stays
//     identical in behaviour to the previous SC version for upstream
//     compatibility (same topic, same message contract).

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/bool.hpp>

#include "dddnav_std_descriptor/STDesc.h"
#include "dddnav_std_descriptor/std_config_loader.h"
#include "dddnav_std_descriptor/std_db_io.h"

namespace dddnav_utils
{

struct PoseRow
{
  double x, y, z, roll, pitch, yaw;
};

// PointXYZIRPYT used by liosam_to_posegraph poses.pcd. Parsed line-by-line
// so we don't have to teach PCL about the custom point type.
static bool loadPoseGraphAscii(const std::string & path,
                               std::vector<PoseRow> & out,
                               rclcpp::Logger log)
{
  std::ifstream f(path);
  if (!f.is_open()) {
    RCLCPP_ERROR(log, "cannot open %s", path.c_str());
    return false;
  }
  std::string line;
  bool in_data = false;
  out.clear();
  while (std::getline(f, line)) {
    if (!in_data) {
      if (line.rfind("DATA", 0) == 0) in_data = true;
      continue;
    }
    if (line.empty()) continue;
    PoseRow row{};
    double intensity = 0.0, time_ = 0.0;
    int read = std::sscanf(line.c_str(), "%lf %lf %lf %lf %lf %lf %lf %lf",
                           &row.x, &row.y, &row.z, &intensity,
                           &row.roll, &row.pitch, &row.yaw, &time_);
    if (read >= 7) out.push_back(row);
  }
  RCLCPP_INFO(log, "loaded %zu poses from %s", out.size(), path.c_str());
  return !out.empty();
}

class StdGlobalInit : public rclcpp::Node
{
public:
  StdGlobalInit()
  : Node("std_global_init")
  {
    declare_parameter<std::string>("std_db_path", "");
    declare_parameter<std::string>("poses_pcd_path", "");
    declare_parameter<std::string>("cloud_topic", "/livox/lidar_liosam_xyzi");
    declare_parameter<std::string>("odom_topic", "/odom_filtered");
    declare_parameter<std::string>("init_pose_topic", "/initial_3d_pose");
    declare_parameter<std::string>("ready_topic", "std_global_init/ready");
    // STD plane-ICP score required to accept a candidate (higher = stricter).
    declare_parameter<double>("std_score_threshold", 0.50);
    declare_parameter<int>("min_consensus_frames", 3);
    declare_parameter<double>("init_cov_xy", 1.0);
    declare_parameter<double>("init_cov_z", 0.25);
    declare_parameter<double>("init_cov_yaw", 0.09);
    declare_parameter<double>("init_cov_rp", 0.04);
    declare_parameter<int>("min_points", 1500);

    // -- Watchdog (in-place re-localisation) ----------------------------
    declare_parameter<bool>("enable_watchdog", true);
    declare_parameter<double>("watchdog_check_hz", 2.0);
    declare_parameter<double>("watchdog_warmup_sec", 8.0);
    declare_parameter<double>("relocate_max_jump_m", 6.0);
    declare_parameter<double>("relocate_max_jump_yaw", 1.05);
    // Watchdog acts only when the STD candidate is (a) strong on its own,
    // and (b) clearly far from the keyframe nearest to MCL's current pose
    // (so we don't fight MCL when it's already correct).
    declare_parameter<double>("relocate_min_score", 0.55);
    declare_parameter<double>("relocate_min_dist_from_live_m", 4.0);
    declare_parameter<int>("relocate_consensus", 4);
    declare_parameter<double>("relocate_holdoff_sec", 10.0);

    std_db_path_      = get_parameter("std_db_path").as_string();
    poses_path_       = get_parameter("poses_pcd_path").as_string();
    cloud_topic_      = get_parameter("cloud_topic").as_string();
    odom_topic_       = get_parameter("odom_topic").as_string();
    init_topic_       = get_parameter("init_pose_topic").as_string();
    score_thresh_     = get_parameter("std_score_threshold").as_double();
    min_consensus_    = get_parameter("min_consensus_frames").as_int();
    cov_xy_           = get_parameter("init_cov_xy").as_double();
    cov_z_            = get_parameter("init_cov_z").as_double();
    cov_yaw_          = get_parameter("init_cov_yaw").as_double();
    cov_rp_           = get_parameter("init_cov_rp").as_double();
    min_points_       = get_parameter("min_points").as_int();
    enable_watchdog_  = get_parameter("enable_watchdog").as_bool();
    watchdog_period_  = 1.0 / std::max(0.1,
                          get_parameter("watchdog_check_hz").as_double());
    watchdog_warmup_  = get_parameter("watchdog_warmup_sec").as_double();
    reloc_max_jump_   = get_parameter("relocate_max_jump_m").as_double();
    reloc_max_yaw_    = get_parameter("relocate_max_jump_yaw").as_double();
    reloc_min_score_  = get_parameter("relocate_min_score").as_double();
    reloc_min_dist_   = get_parameter("relocate_min_dist_from_live_m").as_double();
    reloc_consensus_  = get_parameter("relocate_consensus").as_int();
    reloc_holdoff_    = get_parameter("relocate_holdoff_sec").as_double();
    const auto ready_topic = get_parameter("ready_topic").as_string();

    if (std_db_path_.empty() || poses_path_.empty()) {
      RCLCPP_ERROR(get_logger(),
                   "std_db_path and poses_pcd_path are required");
      bootstrap_done_ = true;
      return;
    }

    // STDescManager config: same parameter prefix as LIO-SAM mapOptimization
    // so the same yaml drives both. skip_near_num goes to 0 — for global
    // init we *want* to match against the entire DB.
    ConfigSetting cfg;
    dddnav_std_descriptor::loadStdConfig(this, cfg);
    cfg.skip_near_num_ = 0;
    cfg.icp_threshold_ = score_thresh_;
    std_mgr_ = std::make_unique<STDescManager>(cfg);

    if (!dddnav_std_descriptor::loadStdDatabase(*std_mgr_, std_db_path_)) {
      RCLCPP_WARN(get_logger(),
                  "no STD db at %s — global init disabled",
                  std_db_path_.c_str());
      bootstrap_done_ = true;
      return;
    }
    if (!loadPoseGraphAscii(poses_path_, poses_, get_logger())) {
      RCLCPP_WARN(get_logger(), "no poses at %s — global init disabled",
                  poses_path_.c_str());
      bootstrap_done_ = true;
      return;
    }
    if (std_mgr_->plane_cloud_vec_.size() != poses_.size()) {
      RCLCPP_WARN(get_logger(),
                  "STD plane-frame count (%zu) != pose count (%zu); "
                  "using min(...) for indexing safety",
                  std_mgr_->plane_cloud_vec_.size(), poses_.size());
    }

    kf_xy_.reserve(poses_.size());
    for (const auto & p : poses_) kf_xy_.emplace_back(p.x, p.y);

    RCLCPP_INFO(get_logger(),
                "STD armed: %zu plane frames, %zu poses, watchdog=%d",
                std_mgr_->plane_cloud_vec_.size(), poses_.size(),
                static_cast<int>(enable_watchdog_));

    pub_init_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      init_topic_, rclcpp::QoS(2).transient_local());
    pub_ready_ = create_publisher<std_msgs::msg::Bool>(
      ready_topic, rclcpp::QoS(1).transient_local());

    rclcpp::QoS cloud_qos(rclcpp::KeepLast(5));
    cloud_qos.best_effort();
    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, cloud_qos,
      std::bind(&StdGlobalInit::onCloud, this, std::placeholders::_1));

    if (enable_watchdog_ && !odom_topic_.empty()) {
      sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(20),
        std::bind(&StdGlobalInit::onOdom, this, std::placeholders::_1));
    }

    bootstrap_started_ = now();
  }

private:
  void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(state_mtx_);
    have_odom_prior_ = true;
    odom_pos_ = Eigen::Vector3d(
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z);
    odom_q_ = Eigen::Quaterniond(
      msg->pose.pose.orientation.w,
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z).normalized();
    odom_stamp_ = msg->header.stamp;
  }

  int nearestKeyframeIdx(const Eigen::Vector3d & p) const
  {
    int best = -1;
    double best_d2 = std::numeric_limits<double>::max();
    for (size_t i = 0; i < kf_xy_.size(); ++i) {
      const double dx = kf_xy_[i].first  - p.x();
      const double dy = kf_xy_[i].second - p.y();
      const double d2 = dx * dx + dy * dy;
      if (d2 < best_d2) { best_d2 = d2; best = static_cast<int>(i); }
    }
    return best;
  }

  static double yawFromQuat(const Eigen::Quaterniond & q)
  {
    const double siny = 2.0 * (q.w() * q.z() + q.x() * q.y());
    const double cosy = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
    return std::atan2(siny, cosy);
  }

  static double angleDiff(double a, double b)
  {
    double d = a - b;
    while (d >  M_PI) d -= 2.0 * M_PI;
    while (d < -M_PI) d += 2.0 * M_PI;
    return std::fabs(d);
  }

  // Compose: world ← T_kf · T_kf_to_query^-1
  // STD's SearchLoop returns transform that takes points in the *query*
  // frame and maps them to the *candidate keyframe* frame. The query frame
  // here is the live LiDAR (body), so the world pose of the body is:
  //   T_world_body = T_world_kf * T_kf_query
  // where T_kf_query is the STD output (loop_t, loop_R).
  geometry_msgs::msg::PoseWithCovarianceStamped buildSeed(
    int kf_idx,
    const Eigen::Vector3d & rel_t,
    const Eigen::Matrix3d & rel_R) const
  {
    const PoseRow & p = poses_[kf_idx];
    Eigen::Quaterniond q_kf =
        Eigen::AngleAxisd(p.yaw,   Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(p.pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(p.roll,  Eigen::Vector3d::UnitX());
    const Eigen::Matrix3d R_kf = q_kf.toRotationMatrix();
    const Eigen::Vector3d t_kf(p.x, p.y, p.z);

    const Eigen::Matrix3d R_world_body = R_kf * rel_R;
    const Eigen::Vector3d t_world_body = R_kf * rel_t + t_kf;
    Eigen::Quaterniond q_world_body(R_world_body);
    q_world_body.normalize();

    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header.stamp = now();
    out.header.frame_id = "map";
    out.pose.pose.position.x = t_world_body.x();
    out.pose.pose.position.y = t_world_body.y();
    out.pose.pose.position.z = t_world_body.z();
    out.pose.pose.orientation.w = q_world_body.w();
    out.pose.pose.orientation.x = q_world_body.x();
    out.pose.pose.orientation.y = q_world_body.y();
    out.pose.pose.orientation.z = q_world_body.z();
    out.pose.covariance[0 * 6 + 0] = cov_xy_;
    out.pose.covariance[1 * 6 + 1] = cov_xy_;
    out.pose.covariance[2 * 6 + 2] = cov_z_;
    out.pose.covariance[3 * 6 + 3] = cov_rp_;
    out.pose.covariance[4 * 6 + 4] = cov_rp_;
    out.pose.covariance[5 * 6 + 5] = cov_yaw_;
    return out;
  }

  // Run STD search against the database. Returns false if no candidate
  // crosses the configured score threshold.
  bool searchOnce(pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
                  int & cand_idx_out,
                  double & score_out,
                  Eigen::Vector3d & rel_t_out,
                  Eigen::Matrix3d & rel_R_out)
  {
    std::vector<STDesc> stds;
    std_mgr_->GenerateSTDescs(cloud, stds);
    if (stds.empty()) return false;

    std::pair<int, double> result(-1, 0.0);
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> rel;
    std::vector<std::pair<STDesc, STDesc>> pairs;
    std_mgr_->SearchLoop(stds, result, rel, pairs);

    cand_idx_out = result.first;
    score_out    = result.second;
    rel_t_out    = rel.first;
    rel_R_out    = rel.second;
    return cand_idx_out >= 0;
  }

  void onCloudBootstrap(pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud)
  {
    int idx = -1;
    double score = 0.0;
    Eigen::Vector3d rel_t;
    Eigen::Matrix3d rel_R;
    if (!searchOnce(cloud, idx, score, rel_t, rel_R) ||
        idx >= static_cast<int>(poses_.size())) {
      RCLCPP_DEBUG(get_logger(), "STD bootstrap: no match (best score=%.3f)",
                   score);
      bootstrap_consensus_ = 0;
      bootstrap_idx_ = -1;
      return;
    }

    if (idx == bootstrap_idx_) {
      bootstrap_consensus_ += 1;
    } else {
      bootstrap_idx_ = idx;
      bootstrap_consensus_ = 1;
      bootstrap_rel_t_ = rel_t;
      bootstrap_rel_R_ = rel_R;
    }

    RCLCPP_INFO(get_logger(),
                "STD bootstrap idx=%d score=%.3f (consensus %d/%d)",
                idx, score, bootstrap_consensus_, min_consensus_);

    if (bootstrap_consensus_ < min_consensus_) return;

    pub_init_->publish(buildSeed(idx, bootstrap_rel_t_, bootstrap_rel_R_));
    std_msgs::msg::Bool ready_msg;
    ready_msg.data = true;
    pub_ready_->publish(ready_msg);
    bootstrap_done_ = true;
    last_publish_ = now();
    RCLCPP_INFO(get_logger(),
                "STD bootstrap published kf=%d (%.2f, %.2f, %.2f)",
                idx, poses_[idx].x, poses_[idx].y, poses_[idx].z);
  }

  void onCloudWatchdog(pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud,
                       const rclcpp::Time & stamp)
  {
    if (!enable_watchdog_) return;
    const double since_pub = (stamp - last_publish_).seconds();
    if (since_pub < watchdog_warmup_) return;
    if (since_pub < reloc_holdoff_) return;
    if ((stamp - last_check_).seconds() < watchdog_period_) return;
    last_check_ = stamp;

    Eigen::Vector3d prior_pos;
    double prior_yaw = 0.0;
    bool have_prior = false;
    {
      std::lock_guard<std::mutex> lk(state_mtx_);
      if (have_odom_prior_) {
        prior_pos = odom_pos_;
        prior_yaw = yawFromQuat(odom_q_);
        have_prior = true;
      }
    }
    if (!have_prior) return;

    const int live_kf = nearestKeyframeIdx(prior_pos);
    if (live_kf < 0) return;

    int cand_idx = -1;
    double score = 0.0;
    Eigen::Vector3d rel_t;
    Eigen::Matrix3d rel_R;
    if (!searchOnce(cloud, cand_idx, score, rel_t, rel_R) ||
        cand_idx >= static_cast<int>(poses_.size()) ||
        score < reloc_min_score_) {
      reloc_streak_ = 0;
      reloc_idx_ = -1;
      return;
    }

    // Candidate keyframe must be far from where MCL currently sits (no
    // point re-bootstrapping to nearby kf — STD's own ICP would be
    // re-asserting the same place).
    const PoseRow & cp = poses_[cand_idx];
    const PoseRow & lp = poses_[live_kf];
    const double d_live = std::hypot(cp.x - lp.x, cp.y - lp.y);
    if (d_live < reloc_min_dist_) {
      reloc_streak_ = 0;
      reloc_idx_ = -1;
      return;
    }

    // ... but also plausibly close to the odom prior (so we don't flip
    // across the map under aliasing).
    const double dxy = std::hypot(cp.x - prior_pos.x(), cp.y - prior_pos.y());
    if (dxy > reloc_max_jump_) {
      reloc_streak_ = 0;
      reloc_idx_ = -1;
      return;
    }

    // Yaw plausibility: world-frame yaw computed from the candidate kf and
    // STD's relative rotation should be near the odom prior.
    Eigen::Quaterniond q_kf =
        Eigen::AngleAxisd(cp.yaw,   Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(cp.pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(cp.roll,  Eigen::Vector3d::UnitX());
    const Eigen::Matrix3d R_world_body = q_kf.toRotationMatrix() * rel_R;
    const double cand_yaw =
        yawFromQuat(Eigen::Quaterniond(R_world_body).normalized());
    if (angleDiff(cand_yaw, prior_yaw) > reloc_max_yaw_) {
      reloc_streak_ = 0;
      reloc_idx_ = -1;
      return;
    }

    if (cand_idx == reloc_idx_) {
      reloc_streak_ += 1;
    } else {
      reloc_idx_ = cand_idx;
      reloc_streak_ = 1;
      reloc_rel_t_ = rel_t;
      reloc_rel_R_ = rel_R;
    }

    RCLCPP_INFO(get_logger(),
                "STD watchdog: live_kf=%d cand=%d score=%.3f d_live=%.2f "
                "dxy_prior=%.2f streak=%d/%d",
                live_kf, cand_idx, score, d_live, dxy,
                reloc_streak_, reloc_consensus_);

    if (reloc_streak_ < reloc_consensus_) return;

    pub_init_->publish(buildSeed(cand_idx, reloc_rel_t_, reloc_rel_R_));
    last_publish_ = stamp;
    reloc_streak_ = 0;
    reloc_idx_ = -1;
    RCLCPP_WARN(get_logger(),
                "STD re-localised: kf=%d (%.2f, %.2f, %.2f) (was kf=%d, "
                "score=%.3f, d_live=%.2f)",
                cand_idx, cp.x, cp.y, cp.z, live_kf, score, d_live);
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*msg, *cloud);
    if (static_cast<int>(cloud->size()) < min_points_) return;

    if (!bootstrap_done_) {
      onCloudBootstrap(cloud);
    } else {
      onCloudWatchdog(cloud, msg->header.stamp);
    }
  }

  // ---- members ---------------------------------------------------------
  std::unique_ptr<STDescManager> std_mgr_;
  std::vector<PoseRow> poses_;
  std::vector<std::pair<double, double>> kf_xy_;
  std::string std_db_path_, poses_path_, cloud_topic_, odom_topic_, init_topic_;
  double score_thresh_{0.5};
  int min_consensus_{3};
  double cov_xy_{1.0}, cov_z_{0.25}, cov_yaw_{0.09}, cov_rp_{0.04};
  int min_points_{1500};

  int bootstrap_idx_{-1};
  int bootstrap_consensus_{0};
  Eigen::Vector3d bootstrap_rel_t_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d bootstrap_rel_R_{Eigen::Matrix3d::Identity()};
  bool bootstrap_done_{false};
  rclcpp::Time bootstrap_started_;

  bool enable_watchdog_{true};
  double watchdog_period_{0.5};
  double watchdog_warmup_{8.0};
  double reloc_max_jump_{6.0};
  double reloc_max_yaw_{1.05};
  double reloc_min_score_{0.55};
  double reloc_min_dist_{4.0};
  int    reloc_consensus_{4};
  double reloc_holdoff_{10.0};
  rclcpp::Time last_check_{rclcpp::Time(0, 0, RCL_ROS_TIME)};
  rclcpp::Time last_publish_{rclcpp::Time(0, 0, RCL_ROS_TIME)};
  int reloc_idx_{-1};
  int reloc_streak_{0};
  Eigen::Vector3d reloc_rel_t_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d reloc_rel_R_{Eigen::Matrix3d::Identity()};

  std::mutex state_mtx_;
  bool have_odom_prior_{false};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond odom_q_{Eigen::Quaterniond::Identity()};
  rclcpp::Time odom_stamp_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_init_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ready_;
};

}  // namespace dddnav_utils

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dddnav_utils::StdGlobalInit>());
  rclcpp::shutdown();
  return 0;
}
