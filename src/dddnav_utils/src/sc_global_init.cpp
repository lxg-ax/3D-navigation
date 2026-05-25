// Copyright (c) 2024, DDDMobileRobot. BSD-3-Clause.
//
// sc_global_init: Scan Context based global localisation bootstrap +
// in-place re-localisation watchdog.
//
// Two modes share the same Scan Context (SC) database that LIO-SAM dumps at
// mapping time (`<map_dir>/lio_sam/sc_db.bin`) plus the keyframe poses
// (`<map_dir>/poses.pcd`):
//
//   1. Bootstrap (no MCL fix yet): on every LiDAR frame we build the body
//      frame SC descriptor, search the DB and require N consecutive frames
//      to agree before publishing /initial_3d_pose.
//
//   2. Watchdog (MCL has been fixing the pose for a while): the same search
//      runs at low rate. The candidate is also gated against the latest
//      fused pose (`/odom_filtered`) so we never flip into a far-away match
//      caused by a geometrically repetitive environment (long corridors,
//      symmetric atria). If the descriptor distance stays clearly above the
//      live MCL keyframe descriptor for `relocate_consensus` consecutive
//      checks AND the SC candidate is within `relocate_max_jump_m` of the
//      odom prior, we re-publish /initial_3d_pose so MCL bootstraps from
//      scratch (kidnap, drift to wrong floor, MCL got stuck in the wrong
//      mode, etc).
//
// Notes
//   * Column shift only recovers yaw, so we copy roll/pitch from the matched
//     keyframe directly. That is acceptable for ground robots (Mid360 stays
//     close to level) — MCL refines the residual roll/pitch in seconds.
//   * Every published initial pose carries an inflated covariance (~1m
//     position, ~0.3rad yaw) so MCL spreads particles around the seed
//     before locking in.
//   * The watchdog is OFF when `enable_watchdog=false`. Bootstrap stays
//     identical to the previous behaviour for backwards compatibility.

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

#include "ScanContext.hpp"

namespace dddnav_utils
{

struct PoseRow
{
  double x, y, z, roll, pitch, yaw;
};

// PointXYZIRPYT used by liosam_to_posegraph poses.pcd. We don't pull the
// custom PCL point type in here — read the binary fields directly. Layout
// produced by `write_poses_pcd` is ASCII; we parse line-by-line to avoid
// having to teach PCL about the custom type.
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

class ScGlobalInit : public rclcpp::Node
{
public:
  ScGlobalInit()
  : Node("sc_global_init")
  {
    declare_parameter<std::string>("sc_db_path", "");
    declare_parameter<std::string>("poses_pcd_path", "");
    declare_parameter<std::string>("cloud_topic", "/livox/lidar_liosam_xyzi");
    declare_parameter<std::string>("odom_topic", "/odom_filtered");
    declare_parameter<std::string>("init_pose_topic", "/initial_3d_pose");
    declare_parameter<std::string>("ready_topic", "sc_global_init/ready");
    declare_parameter<double>("sc_dist_threshold", 0.30);
    declare_parameter<int>("min_consensus_frames", 3);
    declare_parameter<double>("init_cov_xy", 1.0);    // m^2
    declare_parameter<double>("init_cov_z", 0.25);
    declare_parameter<double>("init_cov_yaw", 0.09);  // ~0.3 rad
    declare_parameter<double>("init_cov_rp", 0.04);   // ~0.2 rad
    declare_parameter<int>("min_points", 1500);

    // -- Watchdog (in-place re-localisation) ----------------------------
    declare_parameter<bool>("enable_watchdog", true);
    // How often the watchdog evaluates a frame (Hz). LiDAR is 10 Hz; 2 Hz
    // is plenty and cheap.
    declare_parameter<double>("watchdog_check_hz", 2.0);
    // After bootstrap we wait this long before the watchdog starts, giving
    // MCL time to converge from the seed.
    declare_parameter<double>("watchdog_warmup_sec", 8.0);
    // Spatial gate around the latest /odom_filtered pose. SC candidates
    // farther than this from the prior are rejected to avoid the
    // long-corridor / symmetric-atrium aliasing problem.
    declare_parameter<double>("relocate_max_jump_m", 6.0);
    declare_parameter<double>("relocate_max_jump_yaw", 1.05);  // ~60deg
    // Delta in SC distance: we trigger a re-localisation only if
    //   best_db_dist < live_kf_dist - relocate_min_delta
    // i.e. the fresh candidate has to be clearly better than the keyframe
    // closest to the current MCL pose.
    declare_parameter<double>("relocate_min_delta", 0.05);
    // Bound the alternative match itself — it still has to look like a
    // strong place recognition hit, not just the least-bad row in the DB.
    declare_parameter<double>("relocate_max_candidate_dist", 0.25);
    // How many consecutive watchdog ticks must agree before we re-publish.
    declare_parameter<int>("relocate_consensus", 4);
    // Hold-off after a successful re-localisation (sec) so MCL has room.
    declare_parameter<double>("relocate_holdoff_sec", 10.0);

    sc_db_path_       = get_parameter("sc_db_path").as_string();
    poses_path_       = get_parameter("poses_pcd_path").as_string();
    cloud_topic_      = get_parameter("cloud_topic").as_string();
    odom_topic_       = get_parameter("odom_topic").as_string();
    init_topic_       = get_parameter("init_pose_topic").as_string();
    sc_thresh_        = get_parameter("sc_dist_threshold").as_double();
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
    reloc_min_delta_  = get_parameter("relocate_min_delta").as_double();
    reloc_max_cand_   = get_parameter("relocate_max_candidate_dist").as_double();
    reloc_consensus_  = get_parameter("relocate_consensus").as_int();
    reloc_holdoff_    = get_parameter("relocate_holdoff_sec").as_double();
    const auto ready_topic = get_parameter("ready_topic").as_string();

    if (sc_db_path_.empty() || poses_path_.empty()) {
      RCLCPP_ERROR(get_logger(),
                   "sc_db_path and poses_pcd_path are required");
      bootstrap_done_ = true;        // nothing to do — leave it to the operator
      return;
    }
    if (!sc_.loadDescriptors(sc_db_path_)) {
      RCLCPP_WARN(get_logger(), "no SC db at %s — global init disabled",
                  sc_db_path_.c_str());
      bootstrap_done_ = true;
      return;
    }
    if (!loadPoseGraphAscii(poses_path_, poses_, get_logger())) {
      RCLCPP_WARN(get_logger(), "no poses at %s — global init disabled",
                  poses_path_.c_str());
      bootstrap_done_ = true;
      return;
    }
    if (sc_.size() != static_cast<int>(poses_.size())) {
      RCLCPP_WARN(get_logger(),
                  "SC count (%d) != pose count (%zu); using min(...)",
                  sc_.size(), poses_.size());
    }

    // Pre-compute keyframe centroids for spatial nearest-keyframe lookup
    // used by the watchdog (linear scan is fine for typical maps of a few
    // thousand keyframes; we run at 2Hz).
    kf_xy_.reserve(poses_.size());
    for (const auto & p : poses_) {
      kf_xy_.emplace_back(p.x, p.y);
    }

    RCLCPP_INFO(get_logger(),
                "SC armed: %d descriptors, %zu poses, watchdog=%d",
                sc_.size(), poses_.size(),
                static_cast<int>(enable_watchdog_));

    pub_init_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      init_topic_, rclcpp::QoS(2).transient_local());
    pub_ready_ = create_publisher<std_msgs::msg::Bool>(
      ready_topic, rclcpp::QoS(1).transient_local());

    rclcpp::QoS cloud_qos(rclcpp::KeepLast(5));
    cloud_qos.best_effort();
    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, cloud_qos,
      std::bind(&ScGlobalInit::onCloud, this, std::placeholders::_1));

    if (enable_watchdog_ && !odom_topic_.empty()) {
      sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::QoS(20),
        std::bind(&ScGlobalInit::onOdom, this, std::placeholders::_1));
    }

    bootstrap_started_ = now();
  }

private:
  // Thread-safe write of the latest fused pose (filled by /odom_filtered).
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

  // Index of the keyframe nearest to the latest odom prior (xy-only).
  // Returns -1 if no prior is available.
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
    // Tait-Bryan ZYX yaw (atan2(2(wz+xy), 1-2(yy+zz))).
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

  // Build the seed PoseWithCovarianceStamped (frame "map") for the matched
  // keyframe + descriptor column shift. Caller publishes it.
  geometry_msgs::msg::PoseWithCovarianceStamped buildSeed(
    int kf_idx, int shift) const
  {
    const PoseRow & p = poses_[kf_idx];
    const double yaw_correction = ScanContext::sectorToYaw(shift);

    Eigen::Quaterniond q =
        Eigen::AngleAxisd(p.yaw + yaw_correction, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(p.pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(p.roll,  Eigen::Vector3d::UnitX());
    q.normalize();

    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header.stamp = now();
    out.header.frame_id = "map";
    out.pose.pose.position.x = p.x;
    out.pose.pose.position.y = p.y;
    out.pose.pose.position.z = p.z;
    out.pose.pose.orientation.w = q.w();
    out.pose.pose.orientation.x = q.x();
    out.pose.pose.orientation.y = q.y();
    out.pose.pose.orientation.z = q.z();
    out.pose.covariance[0 * 6 + 0] = cov_xy_;
    out.pose.covariance[1 * 6 + 1] = cov_xy_;
    out.pose.covariance[2 * 6 + 2] = cov_z_;
    out.pose.covariance[3 * 6 + 3] = cov_rp_;
    out.pose.covariance[4 * 6 + 4] = cov_rp_;
    out.pose.covariance[5 * 6 + 5] = cov_yaw_;
    return out;
  }

  // Bootstrap path: untouched semantics, just refactored to share helpers.
  void onCloudBootstrap(const pcl::PointCloud<SCPointType>::Ptr & cloud)
  {
    auto desc = sc_.makeDescriptor(cloud);
    auto best = sc_.detectLoopClosure(desc, /*currentIdx=*/-1,
                                      /*excludeRecent=*/0,
                                      sc_thresh_);
    const int idx = best.first;
    const double dist = best.second;
    if (idx < 0 || idx >= static_cast<int>(poses_.size())) {
      RCLCPP_DEBUG(get_logger(),
                   "no SC match (best dist=%.3f, thresh=%.3f)",
                   dist, sc_thresh_);
      bootstrap_consensus_ = 0;
      bootstrap_idx_ = -1;
      return;
    }

    if (idx == bootstrap_idx_) {
      bootstrap_consensus_ += 1;
    } else {
      bootstrap_idx_ = idx;
      bootstrap_consensus_ = 1;
    }

    RCLCPP_INFO(get_logger(),
                "SC bootstrap idx=%d dist=%.3f (consensus %d/%d)",
                idx, dist, bootstrap_consensus_, min_consensus_);

    if (bootstrap_consensus_ < min_consensus_) return;

    const int shift = sc_.bestShift(desc, sc_.descriptorAt(idx));
    pub_init_->publish(buildSeed(idx, shift));

    std_msgs::msg::Bool ready_msg;
    ready_msg.data = true;
    pub_ready_->publish(ready_msg);
    bootstrap_done_ = true;
    last_publish_ = now();
    RCLCPP_INFO(get_logger(),
                "SC bootstrap published kf=%d (%.2f, %.2f, %.2f)",
                idx, poses_[idx].x, poses_[idx].y, poses_[idx].z);
  }

  // Watchdog path: keep computing SC against the live MCL keyframe and the
  // best DB match. If they diverge persistently AND the candidate is
  // close to the odom prior, re-bootstrap MCL.
  void onCloudWatchdog(const pcl::PointCloud<SCPointType>::Ptr & cloud,
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

    auto desc = sc_.makeDescriptor(cloud);

    // 1. Cosine distance between current scan SC and the live MCL keyframe
    //    SC. If MCL is right this should be small.
    const int live_shift = sc_.bestShift(desc, sc_.descriptorAt(live_kf));
    const double live_dist = computeShiftedDist(desc,
                                                sc_.descriptorAt(live_kf),
                                                live_shift);

    // 2. Best DB match (excluding the live kf to make the comparison fair).
    auto best = sc_.detectLoopClosure(desc, /*currentIdx=*/-1,
                                      /*excludeRecent=*/0,
                                      sc_thresh_);
    const int cand_idx = best.first;
    const double cand_dist = best.second;

    // No candidate or candidate too weak — nothing to act on.
    if (cand_idx < 0 || cand_dist > reloc_max_cand_) {
      reloc_streak_ = 0;
      reloc_idx_ = -1;
      return;
    }

    // Candidate must be clearly better than the live kf SC distance.
    if (cand_dist + reloc_min_delta_ >= live_dist) {
      reloc_streak_ = 0;
      reloc_idx_ = -1;
      return;
    }

    // Spatial gate: candidate within plausible jump from the odom prior.
    const PoseRow & cp = poses_[cand_idx];
    const double dxy = std::hypot(cp.x - prior_pos.x(), cp.y - prior_pos.y());
    if (dxy > reloc_max_jump_) {
      reloc_streak_ = 0;
      reloc_idx_ = -1;
      return;
    }
    // Yaw gate: SC shift gives us the candidate yaw correction.
    const int cand_shift = sc_.bestShift(desc, sc_.descriptorAt(cand_idx));
    const double cand_yaw = cp.yaw + ScanContext::sectorToYaw(cand_shift);
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
    }

    RCLCPP_INFO(get_logger(),
                "SC watchdog: live=%d (d=%.3f) cand=%d (d=%.3f, dxy=%.2f) "
                "streak=%d/%d",
                live_kf, live_dist, cand_idx, cand_dist, dxy,
                reloc_streak_, reloc_consensus_);

    if (reloc_streak_ < reloc_consensus_) return;

    auto seed = buildSeed(cand_idx, cand_shift);
    pub_init_->publish(seed);
    last_publish_ = stamp;
    reloc_streak_ = 0;
    reloc_idx_ = -1;
    RCLCPP_WARN(get_logger(),
                "SC re-localised: kf=%d (%.2f, %.2f, %.2f) (was kf=%d, "
                "live d=%.3f, cand d=%.3f)",
                cand_idx, cp.x, cp.y, cp.z, live_kf, live_dist, cand_dist);
  }

  // Re-derive the cosine distance for a known shift (mirrors ScanContext's
  // private helper; cheap, ~60 column dot-products).
  double computeShiftedDist(const ScanContext::Descriptor & a,
                            const ScanContext::Descriptor & b,
                            int shift) const
  {
    const int N = ScanContext::NUM_SECTOR;
    double sum = 0.0;
    int valid = 0;
    for (int j = 0; j < N; ++j) {
      const int js = (j + shift) % N;
      const Eigen::VectorXd ca = a.col(j);
      const Eigen::VectorXd cb = b.col(js);
      const double na = ca.norm();
      const double nb = cb.norm();
      if (na < 1e-6 || nb < 1e-6) continue;
      double cos = ca.dot(cb) / (na * nb);
      cos = std::min(1.0, std::max(-1.0, cos));
      sum += (1.0 - cos);
      valid += 1;
    }
    return (valid > 0) ? sum / valid : 1.0;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    pcl::PointCloud<SCPointType>::Ptr cloud(new pcl::PointCloud<SCPointType>());
    pcl::fromROSMsg(*msg, *cloud);
    if (static_cast<int>(cloud->size()) < min_points_) {
      return;
    }

    if (!bootstrap_done_) {
      onCloudBootstrap(cloud);
    } else {
      onCloudWatchdog(cloud, msg->header.stamp);
    }
  }

  // ---- members ---------------------------------------------------------
  ScanContext sc_;
  std::vector<PoseRow> poses_;
  std::vector<std::pair<double, double>> kf_xy_;
  std::string sc_db_path_, poses_path_, cloud_topic_, odom_topic_, init_topic_;
  double sc_thresh_{0.3};
  int min_consensus_{3};
  double cov_xy_{1.0}, cov_z_{0.25}, cov_yaw_{0.09}, cov_rp_{0.04};
  int min_points_{1500};

  // Bootstrap state.
  int bootstrap_idx_{-1};
  int bootstrap_consensus_{0};
  bool bootstrap_done_{false};
  rclcpp::Time bootstrap_started_;

  // Watchdog config + state.
  bool enable_watchdog_{true};
  double watchdog_period_{0.5};
  double watchdog_warmup_{8.0};
  double reloc_max_jump_{6.0};
  double reloc_max_yaw_{1.05};
  double reloc_min_delta_{0.05};
  double reloc_max_cand_{0.25};
  int    reloc_consensus_{4};
  double reloc_holdoff_{10.0};
  rclcpp::Time last_check_{rclcpp::Time(0, 0, RCL_ROS_TIME)};
  rclcpp::Time last_publish_{rclcpp::Time(0, 0, RCL_ROS_TIME)};
  int reloc_idx_{-1};
  int reloc_streak_{0};

  // Latest fused-pose prior (used by watchdog).
  std::mutex state_mtx_;
  bool have_odom_prior_{false};
  Eigen::Vector3d odom_pos_{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond odom_q_{Eigen::Quaterniond::Identity()};
  rclcpp::Time odom_stamp_;

  // ROS handles.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_init_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_ready_;
};

}  // namespace dddnav_utils

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dddnav_utils::ScGlobalInit>());
  rclcpp::shutdown();
  return 0;
}
