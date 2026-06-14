// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#ifndef PERCEPTION_3D_NOTHING_LAYER_H_
#define PERCEPTION_3D_NOTHING_LAYER_H_

#include <perception_3d/sensor.h>

namespace perception_3d
{

class NothingLayer: public Sensor{

  public:
    NothingLayer();
    ~NothingLayer();
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
    
};

}//end of name space

#endif