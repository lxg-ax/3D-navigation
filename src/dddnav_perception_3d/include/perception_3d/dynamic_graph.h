// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#ifndef PERCEPTIO_3D_DYNAMIC_GRAPH_H_
#define PERCEPTIO_3D_DYNAMIC_GRAPH_H_

/*For graph*/
#include <unordered_map>
#include <set>
#include <queue> 

//@ where second is the distance to obstacle, smaller closer to obstacle
typedef std::unordered_map<unsigned int, double> dgraph_t;

namespace perception_3d
{

class DynamicGraph{

  public:

    void setValue(unsigned key, double distance);
    void clearValue(unsigned key, double distance);
    double getValue(const unsigned int index);
    void clear();
    void initial(std::size_t n, double max_obstacle_distance);
    unsigned long getdGraphSize(){return graph_.size();}
    dgraph_t graph_;
    
  private:
    

};

}

#endif
