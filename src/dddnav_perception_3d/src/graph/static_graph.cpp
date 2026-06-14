// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

//@For graph
#include <perception_3d/static_graph.h>

namespace perception_3d
{

StaticGraph::StaticGraph(){
  graph_ptr_ = new graph_t();
}

StaticGraph::~StaticGraph(){
  if(graph_ptr_)
    delete graph_ptr_;
}

void StaticGraph::insertNode(unsigned int node, edge_t& a_edge){
  (*graph_ptr_)[node].insert(a_edge);
}

void StaticGraph::insertWeight(unsigned int node, float weight){
  node_weight_[node] = weight;
}

graph_t* StaticGraph::getGraphPtr(){
  return graph_ptr_;
}

edges_t StaticGraph::getEdge(unsigned int node){
  return (*graph_ptr_)[node];
}

float StaticGraph::getNodeWeight(unsigned int node){
  return node_weight_[node];
}

void StaticGraph::clear(){
  node_weight_.clear();
  (*graph_ptr_).clear();
}

}