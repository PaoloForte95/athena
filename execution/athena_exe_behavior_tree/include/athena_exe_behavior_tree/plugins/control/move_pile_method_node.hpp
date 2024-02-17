// Copyright (c) 2023 Paolo Forte
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

#ifndef ATHENA_CE_BEHAVIOR_TREE__PLUGINS__CONTROL__MOVE_PILE_METHOD_NODE_HPP_
#define ATHENA_CE_BEHAVIOR_TREE__PLUGINS__CONTROL__MOVE_PILE_METHOD_NODE_HPP_

#include <string>
#include "behaviortree_cpp_v3/control_node.h"

namespace athena_exe_behavior_tree
{
/**
 * @brief The MovePileMethodNode represents the method move pile in the planning problem.
 
 * It has five children. Upon SUCCESS of the last node in the sequence, this node will halt and return SUCCESS.
 If at any point any of the first four children returns FAILURE, the parent node will also return FAILURE. Upon SUCCESS of the last node in 
 the sequence, this node will halt and return SUCCESS.
 If the last child returns false, it re-ticks previous children.
 
 */
class MovePileMethodNode : public BT::ControlNode
{
public:
  /**
   * @brief A constructor for athena_behavior_tree::MovePileMethodNode
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  MovePileMethodNode(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief A destructor for athena_behavior_tree::MovePileMethodNode
   */
  ~MovePileMethodNode() override = default;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
    };
  }

private:
  unsigned int current_child_idx_;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief The other (optional) override required by a BT action to reset node state
   */
  void halt() override;
};

}  // namespace athena_behavior_tree

#endif
