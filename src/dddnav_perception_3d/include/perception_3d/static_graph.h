// Copyright (c) 2024, DDDMobileRobot
// SPDX-License-Identifier: BSD-3-Clause

#ifndef PERCEPTIO_3D_STATIC_GRAPH_H_
#define PERCEPTIO_3D_STATIC_GRAPH_H_

/*For graph*/
#include <unordered_map>
#include <set>
#include <queue> 

typedef std::pair<unsigned int, float> edge_t;
typedef std::set<edge_t> edges_t;

//@ use unsigned int is because fast triangle return uint32
typedef std::unordered_map<unsigned int, edges_t> graph_t;

namespace perception_3d
{

class StaticGraph{

  public:
    StaticGraph();
    ~StaticGraph();
    void insertNode(unsigned int node, edge_t& a_edge);
    void insertWeight(unsigned int node, float weight);
    graph_t* getGraphPtr();
    edges_t getEdge(unsigned int node);
    float getNodeWeight(unsigned int node);
    void clear();
    unsigned long getSize(){return graph_ptr_->size();};
    unsigned long getNodeWeightSize(){return node_weight_.size();};

  private:
    graph_t* graph_ptr_;
    std::unordered_map<unsigned int, float> node_weight_;

};

}

#endif