// Copyright (c) 2024, DDDMobileRobot. BSD-3-Clause.
//
// C++ port of livox_pc2_to_liosam.py — kept binary-compatible with the Python
// version: same node name, same parameter names, same topic semantics, same
// output point layouts. The Python script remains in scripts/ as a fallback
// for environments without the compiled binary.
//
// Why C++ here:
//   * The Livox driver publishes at 10 Hz with ~20 000 points per frame on
//     Mid360. Doing the byte-level conversion in Python means GIL contention
//     and per-frame allocations of three numpy arrays + two PointCloud2
//     payloads. Under load this jitters the per-point `time` field, which
//     LIO-SAM uses for de-skew, so a delayed bridge directly hurts mapping
//     quality.
//   * The conversion itself is a tight memcpy-like loop, no good reason to
//     pay Python for it.
//
// Output layouts (must match Python sibling exactly):
//   /livox/lidar_liosam       : 32 bytes/point
//                               x(0)/y(4)/z(8)/intensity(16)/ring(20:u16)/time(22:f32)
//                               PCL EIGEN_ALIGN16-compatible padding.
//   /livox/lidar_liosam_xyzi  : 16 bytes/point
//                               x(0)/y(4)/z(8)/intensity(12)
//
// Input layout (Livox xfer_format=0, "CustomMsgPCL2"):
//   x(0)/y(4)/z(8)/intensity(12)/tag(16:u8)/line(17:u8)/timestamp(24:f64)
//   point_step is 32 bytes (driver pads to align timestamp to 8 bytes).

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"

namespace
{

// ---- Output layout constants ------------------------------------------------

constexpr uint32_t LIOSAM_POINT_STEP = 32;
constexpr uint32_t LIOSAM_OFF_X         = 0;
constexpr uint32_t LIOSAM_OFF_Y         = 4;
constexpr uint32_t LIOSAM_OFF_Z         = 8;
constexpr uint32_t LIOSAM_OFF_INTENSITY = 16;
constexpr uint32_t LIOSAM_OFF_RING      = 20;  // uint16
constexpr uint32_t LIOSAM_OFF_TIME      = 22;  // float32

constexpr uint32_t XYZI_POINT_STEP   = 16;
constexpr uint32_t XYZI_OFF_X        = 0;
constexpr uint32_t XYZI_OFF_Y        = 4;
constexpr uint32_t XYZI_OFF_Z        = 8;
constexpr uint32_t XYZI_OFF_INTENSITY = 12;

// ---- Input layout offsets (Livox xfer_format=0) ----------------------------

constexpr uint32_t LIVOX_OFF_X         = 0;
constexpr uint32_t LIVOX_OFF_Y         = 4;
constexpr uint32_t LIVOX_OFF_Z         = 8;
constexpr uint32_t LIVOX_OFF_INTENSITY = 12;
constexpr uint32_t LIVOX_OFF_LINE      = 17;  // uint8 ("ring") inside tag/line/pad block
constexpr uint32_t LIVOX_OFF_TIMESTAMP = 24;  // float64 device time, ns

// ---- PointField helper ------------------------------------------------------

inline sensor_msgs::msg::PointField makeField(
  const std::string & name, uint32_t offset, uint8_t datatype, uint32_t count = 1)
{
  sensor_msgs::msg::PointField f;
  f.name = name;
  f.offset = offset;
  f.datatype = datatype;
  f.count = count;
  return f;
}

}  // namespace

namespace dddnav_utils
{

class LivoxAdapter : public rclcpp::Node
{
public:
  LivoxAdapter()
  : Node("livox_pc2_to_liosam")
  {
    declare_parameter<std::string>("input_topic", "/livox/lidar");
    declare_parameter<std::string>("liosam_output_topic", "/livox/lidar_liosam");
    declare_parameter<std::string>("xyzi_output_topic", "/livox/lidar_liosam_xyzi");

    const auto in_topic   = get_parameter("input_topic").as_string();
    const auto ls_topic   = get_parameter("liosam_output_topic").as_string();
    const auto xyzi_topic = get_parameter("xyzi_output_topic").as_string();

    // Sensor-stream QoS:
    //   * Subscriber on /livox/lidar uses BEST_EFFORT to match the Livox
    //     driver's default sensor-data publisher.
    //   * Publishers downstream use RELIABLE because LIO-SAM's
    //     imageProjection subscribes with RELIABLE QoS (qos_lidar in
    //     utility.hpp). A QoS mismatch silently drops the connection,
    //     leaving imageProjection waiting forever.
    //
    //   /livox/lidar_liosam_xyzi consumers (mcl_feature, sc_global_init)
    //   accept either profile, so RELIABLE is safe across the board.
    rclcpp::QoS sub_qos(rclcpp::KeepLast(5));
    sub_qos.best_effort();
    rclcpp::QoS pub_qos(rclcpp::KeepLast(5));
    pub_qos.reliable();

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      in_topic, sub_qos,
      std::bind(&LivoxAdapter::callback, this, std::placeholders::_1));

    pub_liosam_ = create_publisher<sensor_msgs::msg::PointCloud2>(ls_topic, pub_qos);
    pub_xyzi_   = create_publisher<sensor_msgs::msg::PointCloud2>(xyzi_topic, pub_qos);

    // Pre-build static field descriptors so we don't allocate vector<PointField>
    // every frame.
    liosam_fields_ = {
      makeField("x",         LIOSAM_OFF_X,         sensor_msgs::msg::PointField::FLOAT32),
      makeField("y",         LIOSAM_OFF_Y,         sensor_msgs::msg::PointField::FLOAT32),
      makeField("z",         LIOSAM_OFF_Z,         sensor_msgs::msg::PointField::FLOAT32),
      makeField("intensity", LIOSAM_OFF_INTENSITY, sensor_msgs::msg::PointField::FLOAT32),
      makeField("ring",      LIOSAM_OFF_RING,      sensor_msgs::msg::PointField::UINT16),
      makeField("time",      LIOSAM_OFF_TIME,      sensor_msgs::msg::PointField::FLOAT32),
    };
    xyzi_fields_ = {
      makeField("x",         XYZI_OFF_X,         sensor_msgs::msg::PointField::FLOAT32),
      makeField("y",         XYZI_OFF_Y,         sensor_msgs::msg::PointField::FLOAT32),
      makeField("z",         XYZI_OFF_Z,         sensor_msgs::msg::PointField::FLOAT32),
      makeField("intensity", XYZI_OFF_INTENSITY, sensor_msgs::msg::PointField::FLOAT32),
    };

    RCLCPP_INFO(get_logger(), "%s -> %s + %s",
      in_topic.c_str(), ls_topic.c_str(), xyzi_topic.c_str());
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    const size_t n = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    if (n == 0) {
      return;
    }
    if (msg->point_step == 0 || msg->data.size() < n * msg->point_step) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Malformed Livox cloud: width*height=%zu point_step=%u data_size=%zu",
        n, msg->point_step, msg->data.size());
      return;
    }

    const uint8_t * src = msg->data.data();
    const uint32_t in_step = msg->point_step;

    // Read t0 from the first point's timestamp (float64, ns).
    double t0 = 0.0;
    std::memcpy(&t0, src + LIVOX_OFF_TIMESTAMP, sizeof(double));

    // ---- Build LIO-SAM cloud (32 bytes/point) -----------------------------
    auto out_ls = std::make_unique<sensor_msgs::msg::PointCloud2>();
    out_ls->header = msg->header;
    out_ls->height = 1;
    out_ls->width  = static_cast<uint32_t>(n);
    out_ls->is_bigendian = false;
    out_ls->is_dense = true;
    out_ls->point_step = LIOSAM_POINT_STEP;
    out_ls->row_step   = LIOSAM_POINT_STEP * static_cast<uint32_t>(n);
    out_ls->fields = liosam_fields_;
    out_ls->data.assign(LIOSAM_POINT_STEP * n, 0u);  // zero-init padding bytes

    // ---- Build XYZI cloud (16 bytes/point) --------------------------------
    auto out_xi = std::make_unique<sensor_msgs::msg::PointCloud2>();
    out_xi->header = msg->header;
    out_xi->height = 1;
    out_xi->width  = static_cast<uint32_t>(n);
    out_xi->is_bigendian = false;
    out_xi->is_dense = true;
    out_xi->point_step = XYZI_POINT_STEP;
    out_xi->row_step   = XYZI_POINT_STEP * static_cast<uint32_t>(n);
    out_xi->fields = xyzi_fields_;
    out_xi->data.resize(XYZI_POINT_STEP * n);

    uint8_t * ls_dst = out_ls->data.data();
    uint8_t * xi_dst = out_xi->data.data();

    for (size_t i = 0; i < n; ++i) {
      const uint8_t * sp = src + i * in_step;
      uint8_t * lp = ls_dst + i * LIOSAM_POINT_STEP;
      uint8_t * xp = xi_dst + i * XYZI_POINT_STEP;

      // x/y/z/intensity are 4*float32 → memcpy 16 bytes flat.
      std::memcpy(lp + LIOSAM_OFF_X, sp + LIVOX_OFF_X, 12);                  // x,y,z
      std::memcpy(lp + LIOSAM_OFF_INTENSITY, sp + LIVOX_OFF_INTENSITY, 4);   // intensity

      std::memcpy(xp + XYZI_OFF_X, sp + LIVOX_OFF_X, 12);
      std::memcpy(xp + XYZI_OFF_INTENSITY, sp + LIVOX_OFF_INTENSITY, 4);

      // ring = uint8 line → uint16
      uint16_t ring = static_cast<uint16_t>(sp[LIVOX_OFF_LINE]);
      std::memcpy(lp + LIOSAM_OFF_RING, &ring, sizeof(ring));

      // time = (timestamp_ns - t0_ns) * 1e-9, written as float32 seconds.
      double ts_ns = 0.0;
      std::memcpy(&ts_ns, sp + LIVOX_OFF_TIMESTAMP, sizeof(double));
      float dt_s = static_cast<float>((ts_ns - t0) * 1e-9);
      std::memcpy(lp + LIOSAM_OFF_TIME, &dt_s, sizeof(dt_s));
    }

    pub_liosam_->publish(std::move(out_ls));
    pub_xyzi_->publish(std::move(out_xi));
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_liosam_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_xyzi_;

  std::vector<sensor_msgs::msg::PointField> liosam_fields_;
  std::vector<sensor_msgs::msg::PointField> xyzi_fields_;
};

}  // namespace dddnav_utils

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<dddnav_utils::LivoxAdapter>());
  rclcpp::shutdown();
  return 0;
}
