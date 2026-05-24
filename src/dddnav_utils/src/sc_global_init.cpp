// Copyright (c) 2024, DDDMobileRobot. BSD-3-Clause.
//
// sc_global_init: Scan Context based global localisation bootstrap.
//
// Removes the requirement that the operator hand MCL 3DL a starting pose. On
// startup we:
//   1. Load the Scan Context database produced by LIO-SAM at mapping time
//      (`<map_dir>/lio_sam/sc_db.bin`, body-frame descriptors).
//   2. Load the keyframe pose graph saved by `liosam_to_posegraph`
//      (`<map_dir>/poses.pcd`, fields: x y z intensity roll pitch yaw time;
//      intensity carries the keyframe index, but we use array order anyway).
//   3. Subscribe to a live LiDAR scan, build a body-frame SC descriptor each
//      frame, and search the loaded DB.
//   4. When the best match is below the distance threshold for several
//      consecutive frames *and* the matched index agrees, publish
//      `/initial_3d_pose` with the keyframe pose adjusted by the column-shift
//      yaw offset. After that the node stays alive but stops searching.
//
// Notes
//   * Column shift only recovers yaw, so we copy roll/pitch from the matched
//     keyframe directly. That is acceptable for ground robots (Mid360 stays
//     close to level) — MCL refines the residual roll/pitch in seconds.
//   * The publish covariance is intentionally large (~1m position, ~0.3rad
//     yaw) so MCL spreads particles around the seed before locking in.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
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
    declare_parameter<std::string>("init_pose_topic", "/initial_3d_pose");
    declare_parameter<std::string>("ready_topic", "sc_global_init/ready");
    declare_parameter<double>("sc_dist_threshold", 0.30);
    declare_parameter<int>("min_consensus_frames", 3);
    declare_parameter<double>("init_cov_xy", 1.0);    // m^2
    declare_parameter<double>("init_cov_z", 0.25);
    declare_parameter<double>("init_cov_yaw", 0.09);  // ~0.3 rad
    declare_parameter<double>("init_cov_rp", 0.04);   // ~0.2 rad
    declare_parameter<int>("min_points", 1500);

    sc_db_path_       = get_parameter("sc_db_path").as_string();
    poses_path_       = get_parameter("poses_pcd_path").as_string();
    cloud_topic_      = get_parameter("cloud_topic").as_string();
    init_topic_       = get_parameter("init_pose_topic").as_string();
    sc_thresh_        = get_parameter("sc_dist_threshold").as_double();
    min_consensus_    = get_parameter("min_consensus_frames").as_int();
    cov_xy_           = get_parameter("init_cov_xy").as_double();
    cov_z_            = get_parameter("init_cov_z").as_double();
    cov_yaw_          = get_parameter("init_cov_yaw").as_double();
    cov_rp_           = get_parameter("init_cov_rp").as_double();
    min_points_       = get_parameter("min_points").as_int();
    const auto ready_topic = get_parameter("ready_topic").as_string();

    if (sc_db_path_.empty() || poses_path_.empty()) {
      RCLCPP_ERROR(get_logger(),
                   "sc_db_path and poses_pcd_path are required");
      ready_ = true;        // nothing to do — leave it to the operator
      return;
    }
    if (!sc_.loadDescriptors(sc_db_path_)) {
      RCLCPP_WARN(get_logger(), "no SC db at %s — global init disabled",
                  sc_db_path_.c_str());
      ready_ = true;
      return;
    }
    if (!loadPoseGraphAscii(poses_path_, poses_, get_logger())) {
      RCLCPP_WARN(get_logger(), "no poses at %s — global init disabled",
                  poses_path_.c_str());
      ready_ = true;
      return;
    }
    if (sc_.size() != static_cast<int>(poses_.size())) {
      RCLCPP_WARN(get_logger(),
                  "SC count (%d) != pose count (%zu); using min(...)",
                  sc_.size(), poses_.size());
    }
    RCLCPP_INFO(get_logger(),
                "SC global init armed (%d descriptors, %zu poses)",
                sc_.size(), poses_.size());

    pub_init_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      init_topic_, rclcpp::QoS(2).transient_local());
    pub_ready_ = create_publisher<std_msgs::msg::Bool>(
      ready_topic, rclcpp::QoS(1).transient_local());

    rclcpp::QoS cloud_qos(rclcpp::KeepLast(5));
    cloud_qos.best_effort();
    sub_cloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, cloud_qos,
      std::bind(&ScGlobalInit::onCloud, this, std::placeholders::_1));
  }

private:
  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    if (ready_) return;

    pcl::PointCloud<SCPointType>::Ptr cloud(new pcl::PointCloud<SCPointType>());
    pcl::fromROSMsg(*msg, *cloud);
    if (static_cast<int>(cloud->size()) < min_points_) {
      RCLCPP_DEBUG(get_logger(),
                   "cloud too small (%zu < %d), waiting", cloud->size(),
                   min_points_);
      return;
    }

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
      consensus_count_ = 0;
      consensus_idx_ = -1;
      return;
    }

    if (idx == consensus_idx_) {
      consensus_count_ += 1;
    } else {
      consensus_idx_ = idx;
      consensus_count_ = 1;
    }

    RCLCPP_INFO(get_logger(),
                "SC match idx=%d dist=%.3f (consensus %d/%d)",
                idx, dist, consensus_count_, min_consensus_);

    if (consensus_count_ < min_consensus_) return;

    // Refine yaw via column shift between query and the matched descriptor
    // database. We look up the underlying descriptor through detectLoopClosure
    // again? simpler: re-derive the shift directly.
    const int shift = sc_.bestShift(desc, sc_.descriptorAt(idx));
    const double yaw_correction = ScanContext::sectorToYaw(shift);

    const PoseRow & p = poses_[idx];
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
    // 6x6 row-major: [x y z roll pitch yaw].
    out.pose.covariance[0 * 6 + 0] = cov_xy_;
    out.pose.covariance[1 * 6 + 1] = cov_xy_;
    out.pose.covariance[2 * 6 + 2] = cov_z_;
    out.pose.covariance[3 * 6 + 3] = cov_rp_;
    out.pose.covariance[4 * 6 + 4] = cov_rp_;
    out.pose.covariance[5 * 6 + 5] = cov_yaw_;
    pub_init_->publish(out);

    std_msgs::msg::Bool ready_msg;
    ready_msg.data = true;
    pub_ready_->publish(ready_msg);
    ready_ = true;
    RCLCPP_INFO(get_logger(),
                "Global init published: kf=%d (%.2f, %.2f, %.2f) "
                "yaw=%.2f (corr %.2f)",
                idx, p.x, p.y, p.z, p.yaw + yaw_correction, yaw_correction);
  }

  ScanContext sc_;
  std::vector<PoseRow> poses_;
  std::string sc_db_path_, poses_path_, cloud_topic_, init_topic_;
  double sc_thresh_{0.3};
  int min_consensus_{3};
  double cov_xy_{1.0}, cov_z_{0.25}, cov_yaw_{0.09}, cov_rp_{0.04};
  int min_points_{1500};

  int consensus_idx_{-1};
  int consensus_count_{0};
  bool ready_{false};

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
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
