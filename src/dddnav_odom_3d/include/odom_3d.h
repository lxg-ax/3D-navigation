/*
* BSD 3-Clause License

* Copyright (c) 2024, DDDMobileRobot

* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:

* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.

* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.

* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.

* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef EXAMPLE_ODOM_3D_H_
#define EXAMPLE_ODOM_3D_H_

#include <chrono> // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
 
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp> 

#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


using namespace std::chrono_literals;

class GenericDriveOdom: public rclcpp::Node{

  public:
    GenericDriveOdom():Node("differential_drive_3d_odom_approximation"){init();};
    void cbOdom2D(const nav_msgs::msg::Odometry::SharedPtr msg);
    void cbImu(const sensor_msgs::msg::Imu::SharedPtr msg);
    void pubOdom3D();

  private:
    void init();

    // cb group
    rclcpp::CallbackGroup::SharedPtr cb_group_;

    // Get clock 
    rclcpp::Clock::SharedPtr clock_;

    // sub 2d odom
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_; 
    nav_msgs::msg::Odometry latest_odom_2d_;

    // sub imu
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;   
    sensor_msgs::msg::Imu latest_imu_;
    
    // pub 3d odom
    rclcpp::TimerBase::SharedPtr pub_timer_; 
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_3d_pub_; 

    // tf broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    double imu_roll_;
    double imu_pitch_;
    double imu_yaw_;
    double dt_;
    nav_msgs::msg::Odometry odom_3d_;
    rclcpp::Time current_time_;
    rclcpp::Time last_time_;
    double output_frequency_;

    
};
#endif  // EXAMPLE_ODOM_3D_H_