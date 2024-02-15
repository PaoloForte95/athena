// Copyright (c) 2024 Paolo Forte
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

#ifndef ATHENA_BEHAVIOR_TREE__PLUGINS__DECORATOR__SELECTED_ROBOT_CONTROLLER_HPP_
#define ATHENA_BEHAVIOR_TREE__PLUGINS__DECORATOR__SELECTED_ROBOT_CONTROLLER_HPP_

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/buffer.h"
#include "athena_msgs/msg/method.hpp"
#include "athena_msgs/msg/action.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

typedef std::vector<athena_msgs::msg::Method> Methods;
namespace athena_behavior_tree
{

/**
 * @brief A BT::DecoratorNode that ticks its child every time the current action need to be executed by the robot associated to this decorator.
 */
class SelectedRobotController : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for athena_behavior_tree::SelectedRobotController
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  SelectedRobotController(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("robot_id", "The id of the robot "),
    };
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  rclcpp::Node::SharedPtr node_;
  int robot_id_;
  bool first_time_;
};

}  // namespace athena_behavior_tree

#endif 
