// Copyright (c) 2019 Intel Corporation
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

#ifndef ATHENA_BEHAVIOR_TREE__PLUGINS__CONTROL__PIPELINE_SEQUENCE_HPP_
#define ATHENA_BEHAVIOR_TREE__PLUGINS__CONTROL__PIPELINE_SEQUENCE_HPP_

#include <string>
#include "behaviortree_cpp/control_node.h"
#include "behaviortree_cpp/bt_factory.h"

namespace athena_behavior_tree
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
   * @brief A destructor for athena_behavior_tree::PipelineSequence
   */
  ~PipelineSequence() override = default;

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
}  

#endif  
