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

#ifndef PLAN2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PLANNER_EQUAL_CONDITION_HPP_
#define PLAN2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PLANNER_EQUAL_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"

namespace plan2_behavior_tree
{

/**
 * @brief A BT::ConditionNode that check if the actual action of the execution plan is equal to the one provided as an input 
 * returns SUCCESS when they are equal and FAILURE otherwise
 */
class IsActionEqualCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for plan2_behavior_tree::IsActionEqualCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsActionEqualCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsActionEqualCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "action_name", "Name of the action")
    };
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::string action_name_;
  std::string actual_action_;
  bool first_iteration_;
};

}  // namespace plan2_behavior_tree

#endif  // PLAN2_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PLANNER_EQUAL_CONDITION_HPP_
