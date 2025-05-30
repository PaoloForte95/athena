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


#ifndef ATHENA_BEHAVIOR_TREE__PLUGINS__ACTION__OPTIMIZE_PLAN_ACTION_HPP_
#define ATHENA_BEHAVIOR_TREE__PLUGINS__ACTION__OPTIMIZE_PLAN_ACTION_HPP_

#include <string>
#include <memory>

#include "athena_msgs/msg/plan.hpp"
#include "athena_msgs/action/compute_plan.hpp"
#include "athena_behavior_tree/bt_action_node.hpp"

namespace athena_behavior_tree
{

    /**
 * @brief A athena_behavior_tree::BtActionNode class that wraps athena_behavior_tree::action::OptmizePlanAction
 */
class OptmizePlanAction : public BtActionNode<athena_msgs::action::ComputePlan>
{
public:
  /**
   * @brief A constructor for athena_behavior_tree::ComputePlan
   * @param xml_tag_name Name for the XML tag for this node
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  OptmizePlanAction(
    const std::string & xml_tag_name,
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Function to perform some user-defined operation on tick
   */
  void on_tick() override;

  /**
   * @brief Function to perform some user-defined operation upon successful completion of the action
   */
  BT::NodeStatus on_success() override;

  /**
   * @brief Function to perform some user-defined operation upon abortion of the action
   */
  BT::NodeStatus on_aborted() override;

  /**
   * @brief Function to perform some user-defined operation upon cancelation of the action
   */
  BT::NodeStatus on_cancelled() override;

   /**
   * \brief Override required by the a BT action. Cancel the action and set the path output
   */
  void halt() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::OutputPort<athena_msgs::msg::Plan>("optimized_plan", "The optimized execution plan"),
        BT::InputPort<athena_msgs::msg::Plan>("input_plan", "The planning domain file location"),
        BT::InputPort<int>("alpha", 1, "The value of the linear weight"),     
      });
  }
};

}


#endif