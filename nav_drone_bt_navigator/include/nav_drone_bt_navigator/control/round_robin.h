#pragma once

#include <string>

#include "behaviortree_cpp_v3/control_node.h"

namespace NavigationNodes
{

/** @brief Type of sequence node that ticks children in a round-robin fashion
 *
 * Type of Control Node  | Child Returns Failure | Child Returns Running
 * ---------------------------------------------------------------------
 *  RoundRobin           |    Tick Next Child    | Return Running
 *
 * If the current child return failure, the next child is ticked and if the last child returns
 * failure, the first child is ticked and the cycle continues until a child returns success
 *
 * As an example, let's say this node has 3 children: A, B and C. At the start,
 * they are all IDLE.
 * |    A    |    B    |    C    |
 * --------------------------------
 * |  IDLE   |  IDLE   |  IDLE   |
 * | RUNNING |  IDLE   |  IDLE   |  - at first A gets ticked. Assume it returns RUNNING
 *                                  - RoundRobin returns RUNNING and no other nodes are ticked.
 * | FAILURE | RUNNING |  IDLE   |  - A returns FAILURE so B gets ticked and returns RUNNING
 *                                  - RoundRobin returns RUNNING and C is not ticked yet
 * | FAILURE | SUCCESS |  IDLE   |  - B returns SUCCESS, so RoundRobin halts all children and
 *                                  - returns SUCCESS, next iteration will tick C.
 * | RUNNING |  IDLE   | FAILURE |  - C returns FAILURE, so RoundRobin circles and ticks A.
 *                                  - A returns RUNNING, so RoundRobin returns RUNNING.
 *
 * If all children return FAILURE, RoundRobin will return FAILURE
 * and halt all children, ending the sequence.
 *
 * Usage in XML: <RoundRobin>
 */
class RoundRobinNode : public BT::ControlNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::RoundRobinNode
   * @param name Name for the XML tag for this node
   */
  explicit RoundRobinNode(const std::string & name);

  /**
   * @brief A constructor for nav2_behavior_tree::RoundRobinNode
   * @param name Name for the XML tag for this node
   * @param config BT node configuration
   */
  RoundRobinNode(const std::string & name, const BT::NodeConfiguration & config);

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief The other (optional) override required by a BT action to reset node state
   */
  void halt() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts() {return {};}

private:
  unsigned int current_child_idx_{0};
  unsigned int num_failed_children_{0};
};
  
     
} // Namespace
