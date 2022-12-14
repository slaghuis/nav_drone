#pragma once

#include <stdexcept>
#include <sstream>
#include <string>

#include "behaviortree_cpp_v3/control_node.h"

namespace NavigationNodes
{

/** @brief Type of sequence node that re-ticks previous children when a child returns running
 *
 * Type of Control Node  | Child Returns Failure | Child Returns Running
 * ---------------------------------------------------------------------
 *  PipelineSequence     |      Restart          | Tick All Previous Again
 *
 * Tick All Previous Again means every node up till this one will be reticked. Even
 * if a previous node returns Running, the next node will be reticked.
 *
 * As an example, let's say this node has 3 children: A, B and C. At the start,
 * they are all IDLE.
 * |    A    |    B    |    C    |
 * --------------------------------
 * |  IDLE   |  IDLE   |  IDLE   |
 * | RUNNING |  IDLE   |  IDLE   |  - at first A gets ticked. Assume it returns RUNNING
 *                                  - PipelineSequence returns RUNNING and no other nodes are ticked.
 * | SUCCESS | RUNNING |  IDLE   |  - This time A returns SUCCESS so B gets ticked as well
 *                                  - PipelineSequence returns RUNNING and C is not ticked yet
 * | RUNNING | SUCCESS | RUNNING |  - A gets ticked and returns RUNNING, but since it had previously
 *                                  - returned SUCCESS, PipelineSequence continues on and ticks B.
 *                                  - Since B also returns SUCCESS, C gets ticked this time as well.
 * | RUNNING | SUCCESS | SUCCESS |  - A is still RUNNING, and B returns SUCCESS again. This time C
 *                                  - returned SUCCESS, ending the sequence. PipelineSequence
 *                                  - returns SUCCESS and halts A.
 *
 * If any children at any time had returned FAILURE. PipelineSequence would have returned FAILURE
 * and halted all children, ending the sequence.
 *
 * Usage in XML: <PipelineSequence>
 */
class PipelineSequence : public BT::ControlNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::PipelineSequence
   * @param name Name for the XML tag for this node
   */
  explicit PipelineSequence(const std::string & name);

  /**
   * @brief A constructor for nav2_behavior_tree::PipelineSequence
   * @param name Name for the XML tag for this node
   * @param config BT node configuration
   */
  PipelineSequence(const std::string & name, const BT::NodeConfiguration & config);

  /**
   * @brief The other (optional) override required by a BT action to reset node state
   */
  void halt() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts() {return {};}

protected:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  std::size_t last_child_ticked_ = 0;
};
  
  
     
} // Namespace
