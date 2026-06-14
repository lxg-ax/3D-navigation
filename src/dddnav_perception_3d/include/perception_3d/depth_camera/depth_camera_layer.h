// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#ifndef PERCEPTION_3D_DEPTH_CAMERA_LAYER_H_
#define PERCEPTION_3D_DEPTH_CAMERA_LAYER_H_

#include <perception_3d/sensor.h>
#include <perception_3d/cluster_marking.h>
#include <perception_3d/depth_camera/depth_camera_observation_buffer.hpp>
#include <perception_3d/depth_camera/frustum_utils.h>

namespace perception_3d
{

class DepthCameraLayer: public Sensor{

  public:

    DepthCameraLayer();
    ~DepthCameraLayer();

    void onInitialize();
    void selfClear();
    void selfMark();
    void updateLethalPointCloud();
    pcl::PointCloud<pcl::PointXYZI>::Ptr getObservation();
    pcl::PointCloud<pcl::PointXYZI>::Ptr getLethal();
    void resetdGraph();
    double get_dGraphValue(const unsigned int index);
    bool isCurrent();

  private:
    
    std::shared_ptr<perception_3d::Marking> pct_marking_;
    std::shared_ptr<perception_3d::FrustumUtils> frustum_utils_;

    rclcpp::Clock::SharedPtr clock_;
    
    rclcpp::CallbackGroup::SharedPtr marking_pub_cb_group_;

    rclcpp::TimerBase::SharedPtr marking_pub_timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_current_observation_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_current_segmentation_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_current_projected_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_current_window_marking_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_cleared_window_marking_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_casting_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_gbl_marking_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dGraph_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_frustum_;

    void cbSensor(const sensor_msgs::msg::PointCloud2::SharedPtr msg,
                                    const std::shared_ptr<perception_3d::DepthCameraObservationBuffer>& buffer);
    void aggregatePointCloudFromObservations(const pcl::PointCloud<pcl::PointXYZI>::Ptr& resulting_pcl)  ;
        
    //@ For casting visualization
    void addCastingMarker(const pcl::PointXYZI& pt, size_t id, visualization_msgs::msg::MarkerArray& markerArray);
    
    void pubUpdateLoop();

    std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> sub_pc_map_; 
    std::map<std::string, std::shared_ptr<perception_3d::DepthCameraObservationBuffer>> observation_buffers_;
    
    bool pub_gbl_marking_for_visualization_;
    double pub_gbl_marking_frequency_;
    bool is_local_planner_;
    double resolution_, height_resolution_;
    double segmentation_ignore_ratio_;
    double perception_window_size_; 
    double marking_height_;
    double euclidean_cluster_extraction_tolerance_;
    int euclidean_cluster_extraction_min_cluster_size_;
    geometry_msgs::msg::TransformStamped trans_gbl2b_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_current_window_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_msg_gbl_;
};

}//end of name space
#endif