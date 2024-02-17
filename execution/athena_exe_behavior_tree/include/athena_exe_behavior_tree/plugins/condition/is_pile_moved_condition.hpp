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

#ifndef ATHENA_CE_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PILE_MOVED_HPP_
#define ATHENA_CE_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PILE_MOVED_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/condition_node.h"
#include "nav2_msgs/srv/get_costmap.hpp"

namespace athena_exe_behavior_tree
{

/**
 * @brief A BT::ConditionNode that check if the pile of material has been completely (or partially) moved from the loading position.
 */
class IsPileMovedCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for athena_behavior_tree::IsPileMovedCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsPileMovedCondition(const std::string & condition_name, const BT::NodeConfiguration & conf);

  IsPileMovedCondition() = delete;

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
      BT::InputPort<std::string>("criteria", "NumberCycles", "Criteria to use to check if the pile is completely moved."),
      BT::InputPort<std::string>("global_frame", "map","Global frame"),
      BT::InputPort<std::string>("costmap_topic", "local_costmap/get_costmap","The costmap Topic"),
      BT::InputPort<int>("ticks", 1, "Number of ticks to do (If NumberCycles criteria is selected)"),
      BT::InputPort<double>("threshold", 0.5, "The threshold value for which the position is considered free(If LidarScan criteria is selected)."),
    };
  }

private:

  rclcpp::Node::SharedPtr node_;
  std::string global_frame_, costmap_topic_;
  std::string criteria_;
  int ticks_;
  int current_tick_;
  double threshold_;
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr costmap_client_;
};

}  // namespace 

#endif