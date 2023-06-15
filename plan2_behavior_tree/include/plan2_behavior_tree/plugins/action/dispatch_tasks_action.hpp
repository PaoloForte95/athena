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

#ifndef PLAN2_BEHAVIOR_TREE__PLUGINS__ACTION__DISPATCH_TASKS_ACTION_HPP_
#define PLAN2_BEHAVIOR_TREE__PLUGINS__ACTION__DISPATCH_TASKS_ACTION_HPP_

#include <string>
#include <memory>

#include "plan2_msgs/msg/plan.hpp"
#include "plan2_msgs/msg/action.hpp"
#include "plan2_behavior_tree/bt_action_node.hpp"

namespace plan2_behavior_tree
{
/**
 * @brief A plan2_behavior_tree::BtActionNode class that wraps plan2_behavior_tree::action::DispatchTasksAction
 */
class DispatchTasksAction : public BT::ActionNodeBase
{
    
public:

    typedef std::vector<plan2_msgs::msg::Action> Actions;
    typedef std::vector<int> IDs;

    /**
     * @brief A constructor for plan2_behavior_tree::DispatchTasksAction
     * @param action_name Action name this node creates a client for
     * @param conf BT node configuration
     */
    DispatchTasksAction(
        const std::string & action_name,
        const BT::NodeConfiguration & conf);

    /**
     * @brief The other (optional) override required by a BT action.
     */
    void halt() override {}

    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific ports
     */
    static BT::PortsList providedPorts()
    {
        return 
        {   
            BT::OutputPort<Actions>("concurrent_actions","The set of all actions that can be executed at current step"),
            BT::InputPort<plan2_msgs::msg::Plan>("execution_plan", "The computed execution plan"), 
            BT::InputPort<IDs>("completed_actions","The set of completed actions"),
        };
    }

private:

  rclcpp::Node::SharedPtr node_;
  IDs completed_actions_;
  plan2_msgs::msg::Plan execution_plan_;
  std::map<int, int> action_levels;
  int current_level_, max_level_;
  int robots;
  int actions_count_;

  /**
   * @brief Read the action from the exection plan
   * 
   * @param execution_plan_ The execution plan
   * @return Actions The set of actions defined in the exeuction plan
   */
  Actions readPlan(plan2_msgs::msg::Plan execution_plan_);

  int executableActions(Actions actions);

  plan2_msgs::msg::Action getAction (int actionID);

}; //Class end

}  // namespace plan2_behavior_tree

#endif  // PLAN2_BEHAVIOR_TREE__PLUGINS__ACTION__DISPATCH_TASKS_ACTION_HPP_