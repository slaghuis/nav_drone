// Copyright (c) 2022 Eric Slaghuis
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* ***********************************************************************
 * Functions to enable the path planner to function against a cost planner
 * defined by an Octomap
 * ***********************************************************************/
#pragma once
#include "nav_drone_theta_star_planner/path_finding.hpp"

//The pathfinder is a general algorithm that can be used for mutliple purposes
//So it uses this adaptor
//This adaptor is for octree
//This adapteor requires that any node addressed is defined (either free or occupied, but not unknown)

class CostmapAdaptor: public Pathfinder::PathfinderAdaptor
{
public:

  using NodeId = Pathfinder::NodeId;
  using Cost = Pathfinder::Cost;
  
  CostmapAdaptor(std::shared_ptr<octomap::OcTree> octree, unsigned int depth) : octree_(octree), working_depth_(depth) { }
  
  virtual size_t getNodeCount() const override
  {
    return octree_->getNumLeafNodes();  // Not used in this adaptor
  }

  //return the distance between two nodes
  virtual Cost distance(const NodeId one, const NodeId two) const override
  {
    return one.distance(two);   
  }

  //Return true if there is a direct path between n1 and n2
  virtual bool lineOfSight(const NodeId start, const NodeId end) const override
  {
    octomath::Vector3 dummy;
		octomath::Vector3 direction = end - start;
    
		bool node_hit = octree_->castRay( start, direction, dummy, true, (double)distance(start, end) );
    
    return !node_hit;
  }
  
  virtual double searchResolution() const override
  {
    return  octree_->getResolution() * pow(2, octree_->getTreeDepth() - working_depth_);
  };
  
  // Return a vector of all the neighbors ids and the cost to travel to them
  // In this adaptor we only need to check the four tile neigbors and the cost is always 1  
  virtual std::vector<std::pair<NodeId, Cost>> getNodeNeighbors(const NodeId node) const override
  {
    std::vector<std::pair<NodeId, Cost>> neighbors;
    
    double search_resolution = searchResolution();
    Pathfinder::Cost cost = 10.0;
    
    double x, y, z;    
    x = node.x() - search_resolution;
    y = node.y();
    z = node.z();
    
    octomap::OcTreeKey neighbor_node = octree_->coordToKey(x, y, z, working_depth_);
    NodeId point = octree_->keyToCoord(neighbor_node);
    neighbors.push_back( {point, cost} );
    
    
    x = node.x() + search_resolution;
    y = node.y();
    z = node.z();   
    neighbor_node = octree_->coordToKey(x, y, z, working_depth_);
    point = octree_->keyToCoord(neighbor_node);
    neighbors.push_back( {point, cost} );

    x = node.x();
    y = node.y() - search_resolution;
    z = node.z();    
    neighbor_node = octree_->coordToKey(x, y, z, working_depth_);
    point = octree_->keyToCoord(neighbor_node);
    neighbors.push_back( {point, cost} );

    x = node.x();
    y = node.y() + search_resolution;
    z = node.z();    
    neighbor_node = octree_->coordToKey(x, y, z, working_depth_);
    point = octree_->keyToCoord(neighbor_node);
    neighbors.push_back( {point, cost} );

    cost = 15.0;    // Pay a price for increasing or decreasing altitude

    x = node.x();
    y = node.y();
    z = node.z() - search_resolution;    
    neighbor_node = octree_->coordToKey(x, y, z, working_depth_);
    point = octree_->keyToCoord(neighbor_node);
    neighbors.push_back( {point, cost} );

    x = node.x();
    y = node.y();
    z = node.z() + search_resolution;    
    neighbor_node = octree_->coordToKey(x, y, z, working_depth_);
    point = octree_->keyToCoord(neighbor_node);
    neighbors.push_back( {point, cost} );

    return neighbors;
  }
private:
    std::shared_ptr<octomap::OcTree> octree_;
    
    unsigned int working_depth_ = 3;  // Assume the deepest level at maximum resolution is 0.  Half that is level 1, etc.
};