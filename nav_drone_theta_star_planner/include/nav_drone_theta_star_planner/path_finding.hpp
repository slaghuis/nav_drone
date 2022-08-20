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

/* **********************************************************************************************
   * An implimentation of the Theta Star algorithm as descibed by Nash, Koenig and Tovey 
   * http://idm-lab.org/bib/abstracts/papers/aaai10b.pdf
   * ********************************************************************************************/
#pragma once

#include <vector>     // std::vector
#include <memory>     // std::shared_ptr
#include <algorithm>  // std::sort
#include <functional>
#include <numeric>

class Pathfinder
{
  public:
    using NodeId = octomap::point3d; // The only reference to the costmap specific variables or functions in this code
    using Cost = float;
    // The pathfinder is a general algorithm that can be used for mutliple purposes
    // It uses an adaptor to expose the functions that adresses the costmap (Octomap in this case)
    class PathfinderAdaptor
    {
      public:
        friend Pathfinder;

        virtual size_t getNodeCount() const = 0;
        virtual double searchResolution() const = 0;
        virtual Cost distance(const NodeId one, const NodeId two) const = 0;
        virtual bool lineOfSight(const NodeId n1, const NodeId n2) const = 0;
        virtual std::vector<std::pair<NodeId, Cost>> getNodeNeighbors(const NodeId point) const = 0;
    };

    static constexpr Cost INFINITE_COST = std::numeric_limits<Cost>::max();       // A very big number
    static constexpr const float EPSILON = 0.00001f;                              // Just a small number

    // The heap element stores variables that informs the search algirithm
    struct HeapElement {
      NodeId id;                             // A reference to coordinates of this in the octomap
      Cost g;			                           // Used for tie-breaking
      Cost f;		                             // Initialized to the heuristic distance to the goal when generated.
      std::shared_ptr<HeapElement> parent;	 // Parent of this node.

      HeapElement( NodeId coordinates )
      {
        id = coordinates;
        g  = INFINITE_COST;
      }
      
    };
  
  
    Pathfinder(PathfinderAdaptor& adaptor, Cost weight = 1.0f) : adaptor(adaptor), weight(weight) { }
  
    std::vector<NodeId> search(const NodeId start_point, const NodeId end_point)
    {
      end_point_ = end_point;  // Save as a object variable for common use
      
      open.clear();
      std::shared_ptr<HeapElement> start = std::make_shared<HeapElement>(start_point);
      start->g = 0.0;
      start->f = start->g + adaptor.distance(start->id, end_point_) * weight;
      start->parent = start;
      open.push_back(start);
    
      closed.clear();   
      
      double threshold = adaptor.searchResolution();
    
      while (!open.empty()) {        
        std::shared_ptr<HeapElement> s = open_pop();
        
        if(s->id.distance( end_point ) < threshold) {  
          return reconstructPath(s);
        }  
        
        closed.push_back(s);
        std::vector<std::pair<NodeId, Cost>> list_of_neighbors = adaptor.getNodeNeighbors( s->id );
        for(const auto& neighbor : list_of_neighbors) {
          // Search for neighbor in closed
          if( std::find_if( closed.begin(), closed.end(),
                           [&neighbor](const std::shared_ptr<HeapElement> &a) 
                           {
                             return a->id == neighbor.first;
                           }) == closed.end() ) {  // Neighbor is not in closed
            
            std::vector<std::shared_ptr<HeapElement>>::iterator f = std::find_if( open.begin(), open.end(),
                                                                                 [&neighbor](const std::shared_ptr<HeapElement> &a) 
                                                                                 {
                                                                                    return a->id == neighbor.first;
                                                                                 });
            std::shared_ptr<HeapElement> n;
            if( f == open.end() ) {  // Not found 
              n = std::make_shared<HeapElement>(neighbor.first);
              n->g = INFINITE_COST;
              n->parent = NULL;
              
              // Neighbor is not in open.  Place it there
            } else {

              n = *f;  // Asign the found node
            }
            updateVertex(s, n, neighbor.second, (f != open.end()) );
          }          
        }  
      }
    
      // No valid path found.  Just return an empty list.
      std::vector<NodeId> path;
      return path;               
    }
                                         
  private:
    std::vector<std::shared_ptr<HeapElement>> open;  
    std::vector<std::shared_ptr<HeapElement>> closed;  
  
    NodeId end_point_;
  
    PathfinderAdaptor& adaptor;
    const Cost weight;

    std::shared_ptr<HeapElement> open_pop() {
      // Sort the open list
      std::sort(open.begin(), open.end(),
                []( std::shared_ptr<HeapElement> &a, std::shared_ptr<HeapElement> &b) 
                {
                  if( abs(a->f - b->f) < EPSILON) //nodes are equal, use a tie breaker
                    return a->g < b->g;
                  else
                    return a->f > b->f;
                });  
    
      std::shared_ptr<HeapElement> back = open.back();
      open.pop_back();  // Remove from the list
        
      return back;
    }

    std::vector<NodeId> reconstructPath(std::shared_ptr<HeapElement> s) 
    {
      std::vector<NodeId> path;
      
      while(s != s->parent ) {
        path.push_back(s->id);
        s = s->parent;
      }  
      
      path.push_back(s->id);  // The start node 
      
      std::reverse(path.begin(), path.end());
      
      return path;
    }
  
    void updateVertex(std::shared_ptr<HeapElement>s, std::shared_ptr<HeapElement>n, Cost c, bool found) {
      if (adaptor.lineOfSight(s->parent->id, n->id)) {
        Cost tentative_score = s->parent->g + c;   // The c should be c(parent, negihbor).  This one is c(s, neighbor)
        if( tentative_score < n->g) {
          n->g = tentative_score;
          n->parent = s->parent;
          if( !found ) {
            open.push_back(n);
          }
          n->f = n->g + adaptor.distance(n->id, end_point_);          
        }
      } else {
        Cost tentative_score = s->parent->g + c;   // The c should be c(parent, negihbor).  This one is c(s, neighbor)
        if(tentative_score < n->g) {
          n->g = tentative_score;
          n->parent = s;
          if( !found ) {
            open.push_back(n);
          }
          n->f = n->g + adaptor.distance(n->id, end_point_);
        }
      }
    }
};