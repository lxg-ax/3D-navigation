// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

//@For graph
#include <perception_3d/dynamic_graph.h>

namespace perception_3d
{

void DynamicGraph::setValue(unsigned int key, double distance){
  //@ make sure we set minimum value, we have initialize the graph_, so dont worry memory lookup crash
  graph_[key] = std::min(graph_[key],distance);
}

void DynamicGraph::clearValue(unsigned int key, double distance){
  //@ overwrite the value
  graph_[key] = distance;
}

double DynamicGraph::getValue(const unsigned int index){
  return graph_[index];
}

void DynamicGraph::clear(){
  graph_.clear();
}

void DynamicGraph::initial(std::size_t n, double max_obstacle_distance){
  for(unsigned int i=0;i<=n;i++){
    graph_[i] = max_obstacle_distance;
  }
}

}