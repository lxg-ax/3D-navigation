// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#ifndef PERCEPTION_3D_STACKED_PERCEPTION_H_
#define PERCEPTION_3D_STACKED_PERCEPTION_H_

#include <perception_3d/sensor.h>

/*Debug*/
#include <sys/time.h>
#include <time.h>

/*To iterate the ymal for plugins and also for mutex lock*/
#include <pluginlib/class_loader.hpp>
# include <geometry_msgs/msg/pose_stamped.hpp>

namespace perception_3d
{

class StackedPerception{

  public:

    StackedPerception(const rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr& m_logger);
    ~StackedPerception();

    /*Plugin operations*/
    void addPlugin(std::shared_ptr<Sensor> plugin);
    std::vector<std::shared_ptr<Sensor> >* getPlugins();

    /*Collected all marked pcl*/
    void doClear_then_Mark();
    
    /*Reset the sensors*/
    void resetdGraph();

    /*Get value from d graph. Loop all plugins and return minimum one (closest to obstacles)*/
    double get_min_dGraphValue(const unsigned int index);

    /*Get value from d graph. Loop all plugins and return minimum size of dGraph one (closest to obstacles)*/
    unsigned long getdGraphSize();

    /*Aggregate observation pointers for local planner*/
    void aggregateObservations();

    /*Aggregate lethal for line-of-sight check in global planner*/
    void aggregateLethal();

    /*Collect all opinions and return as a vector*/
    std::vector<perception_3d::PerceptionOpinion> getOpinions();
    
    std::shared_ptr<perception_3d::SharedData> getSharedDataPtr(){return shared_data_;}

    // Provide a typedef to ease future code maintenance
    typedef std::recursive_mutex mutex_t;
    mutex_t* getMutex()
    {
      return access_;
    }

    /*Check sensors are publish topics*/
    bool isSensorOK();
    
  private:
    std::vector<std::shared_ptr<Sensor>> plugins_;

    /*mutex*/
    mutex_t* access_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_prune_plan_;

    std::shared_ptr<SharedData> shared_data_;

    rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr logger_;
};

}//end of name space

#endif