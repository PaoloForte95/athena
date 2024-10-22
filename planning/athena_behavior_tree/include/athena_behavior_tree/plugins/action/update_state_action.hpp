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

#ifndef ATHENA_BEHAVIOR_TREE__PLUGINS__ACTION__UPDATE_STATE_HPP_
#define ATHENA_BEHAVIOR_TREE__PLUGINS__ACTION__UPDATE_STATE_HPP_

#include <string>
#include <memory>

#include "athena_behavior_tree/bt_action_node.hpp"
#include "athena_msgs/action/update_state.hpp"
#include "athena_msgs/msg/action.hpp"
#include "athena_msgs/msg/state.hpp"
typedef std::vector<athena_msgs::msg::Action> Actions;

namespace athena_behavior_tree
{
/**
 * @brief A athena_behavior_tree::BtActionNode class that wraps athena_behavior_tree::action::UpdateStateAction
 */
class UpdateStateAction : public BtActionNode<athena_msgs::action::UpdateState>
{
    
public:
    /**
     * @brief A constructor for athena_behavior_tree::UpdateStateAction
     * @param action_name Action name this node creates a client for
     * @param conf BT node configuration
     */
    UpdateStateAction(
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
            BT::InputPort<athena_msgs::msg::State>("previous_state", "The previous state"), 
            BT::InputPort<std::string>("state_updater", "The state updater to use"),
            BT::OutputPort<athena_msgs::msg::State>("current_state","The updated state"), 
      });
  }
}; 

} 

#endif 