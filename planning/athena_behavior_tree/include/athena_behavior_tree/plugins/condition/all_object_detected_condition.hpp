// Copyright (c) 2025 Paolo Forte
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

#ifndef ATHENA_BEHAVIOR_TREE__PLUGINS__CONDITION__ALL_OBJECT_DETECTED_CONDITION_HPP_
#define ATHENA_BEHAVIOR_TREE__PLUGINS__CONDITION__ALL_OBJECT_DETECTED_CONDITION_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp/condition_node.h"

#include "rclcpp/rclcpp.hpp"


namespace athena_behavior_tree
{

/**
 * @brief A BT::ConditionNode that returns SUCCESS every time the robot
 * travels a specified distance and FAILURE otherwise
 */
class AllObjectDetectedCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::DistanceTraveledCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  AllObjectDetectedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  AllObjectDetectedCondition() = delete;

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
    };
  }

private:

  bool checkListObjects();
private:
  rclcpp::Node::SharedPtr node_;

};

}  

#endif 