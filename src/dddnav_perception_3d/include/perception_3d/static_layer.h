// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#ifndef PERCEPTION_3D_STATIC_LAYER_H_
#define PERCEPTION_3D_STATIC_LAYER_H_

#include <perception_3d/sensor.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
/*Point cloud library*/
#include <pcl/point_types.h>

/*allows us to use pcl::transformPointCloud function*/
#include <tf2_eigen/tf2_eigen.hpp>

/*For pcl::transformPointCloud, dont use #include <pcl/common/transforms.h>*/
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

/*For distance calculation*/
#include <pcl/common/geometry.h>
#include <math.h>

/*Fast triangulation of unordered point clouds*/
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

/*RANSAC*/
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

/*pass through*/
#include <pcl/filters/passthrough.h>

namespace perception_3d
{

class StaticLayer: public Sensor{

  public:
    StaticLayer();
    ~StaticLayer();
    virtual void onInitialize();
    virtual void selfClear();
    virtual void selfMark();
    virtual void updateLethalPointCloud();
    virtual pcl::PointCloud<pcl::PointXYZI>::Ptr getObservation();
    pcl::PointCloud<pcl::PointXYZI>::Ptr getLethal();
    virtual void resetdGraph();
    virtual double get_dGraphValue(const unsigned int index);
    virtual bool isCurrent();

  private:
    
    void ptrInitial();
    void radiusSearchConnection();
    void generateStaticGraph();

    /*call back of the ground*/
    void cbGround(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    /*call back of the map*/
    void cbMap(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /*Subscriber*/
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_ground_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_map_sub_;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dGraph_;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_map_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_ground_;

    bool new_map_, new_ground_;
    rclcpp::CallbackGroup::SharedPtr cbs_group_;
    bool is_local_planner_;

    bool use_adaptive_connection_;
    int adaptive_connection_number_;
    double radius_of_ground_connection_;
    double turning_weight_;
    double intensity_search_radius_;
    double intensity_search_punish_weight_;
    double static_imposing_radius_;
    bool mapping_mode_;
    std::string map_topic_;
    std::string ground_topic_;
    bool is_ground_and_map_being_initialized_once_;
    bool enable_edge_detection_;
    bool generate_static_graph_;
};

}//end of name space

#endif