// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#ifndef PERCEPTION_3D_ROS_H_
#define PERCEPTION_3D_ROS_H_

#include <perception_3d/stacked_perception.h>
#include "pluginlib/class_loader.hpp"

/*ROS API for clear perceptions*/
#include "std_srvs/srv/empty.hpp"

namespace perception_3d
{

class Perception3D_ROS : public rclcpp::Node {
  
  public:
    Perception3D_ROS(std::string name);
    ~Perception3D_ROS();
    double get_min_dGraphValue(const unsigned int index);
    std::shared_ptr<GlobalUtils> getGlobalUtils(){return gbl_utils_;}
    StackedPerception* getStackedPerception(){return stacked_perception_;}  
    void getGlobalPose(geometry_msgs::msg::TransformStamped& gbl_pose);
    void getGlobalPose(geometry_msgs::msg::PoseStamped& gbl_pose);
    std::shared_ptr<perception_3d::SharedData> getSharedDataPtr();
    void initial();
    unsigned long getdGraphSize();

  private:

    /*For plugin loader*/
    pluginlib::ClassLoader<Sensor> plugin_loader_;
    std::vector<std::string> plugins_;
    rclcpp::TimerBase::SharedPtr sensors_update_loop_timer_;
    rclcpp::TimerBase::SharedPtr dgraph_publish_timer_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Time mark_and_clear_start_time_;

    double sensors_collected_frequency_;
    double dgraph_publish_frequency_;
    void sensorsUpdateLoop();
    void dGraphPublishLoop();
    
    void clearPerceptionsSrv(const std::shared_ptr<std_srvs::srv::Empty::Request> req, 
                        std::shared_ptr<std_srvs::srv::Empty::Response> resp);

    rclcpp::CallbackGroup::SharedPtr tf_listener_group_;
    rclcpp::CallbackGroup::SharedPtr clear_perception_and_timer_group_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dGraph_;

  protected:

    StackedPerception* stacked_perception_;
    
    std::shared_ptr<tf2_ros::TransformListener> tfl_;
    std::shared_ptr<tf2_ros::Buffer> tf2Buffer_;  ///< @brief Used for transforming point clouds
    std::string name_;
    std::string global_frame_;  ///< @brief The global frame
    std::string robot_base_frame_;  ///< @brief The frame_id of the robot base

    /*For path planning parameter, similar to costmap_2d*/
    double max_obstacle_distance_;
    double inscribed_radius_;
    double inflation_descending_rate_;
    double inflation_radius_;
    
    double transform_tolerance_;  ///< timeout before transform errors

    std::shared_ptr<GlobalUtils> gbl_utils_; ///< passing global frame, tf_buffer to plugins

    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_perceptions_srv_;
};

}//end of name space

#endif
