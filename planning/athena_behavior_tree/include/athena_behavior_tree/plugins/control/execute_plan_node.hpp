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

#ifndef ATHENA_BEHAVIOR_TREE__PLUGINS__CONTROL__EXECUTE_PLAN_NODE_HPP_
#define ATHENA_BEHAVIOR_TREE__PLUGINS__CONTROL__EXECUTE_PLAN_NODE_HPP_

#include <string>
#include "behaviortree_cpp/control_node.h"

namespace athena_behavior_tree
{
/**
 * @brief The ExecutePlanNode has only two children and returns SUCCESS if and only if the first child
 * returns SUCCESS.
 *
 * - If the first child returns FAILURE, the second child will be executed.  After that the first
 * child is executed again if the second child returns SUCCESS.
 *
 * - If the first or second child returns RUNNING, this node returns RUNNING.
 *
 * - If the second child returns FAILURE, this control node will stop the loop and returns FAILURE.
 *
 */
class ExecutePlanNode : public BT::ControlNode
{
public:
  /**
   * @brief A constructor for athena_behavior_tree::ExecutePlanNode
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  ExecutePlanNode(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief A destructor for athena_behavior_tree::ExecutePlanNode
   */
  ~ExecutePlanNode() override = default;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<int>("plan_length", 1, "Length of the execution plan")
    };
  }

private:
  unsigned int current_child_idx_;
  int plan_length_;
  int actions_count_;

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

} 

#endif 
