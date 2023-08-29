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


#ifndef PLAN2_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_ACTION_HPP_
#define PLAN2_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_ACTION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "plan2_behavior_tree/bt_action_node.hpp"
#include "plan2_msgs/msg/action.hpp"

namespace plan2_behavior_tree
{

enum class ActionStatus
{
  UNKNOWN = 0,
  PROCESSING = 1,
  FAILED = 2,
  SUCCEEDED = 3
};


/**
 * @brief Create an action client to send the current action of the execution plan.
 * 
 */

class SendMoveAction : public BT::ActionNodeBase
{
public:
  /**
   * @brief A constructor for plan2_behavior_tree::SendMoveAction
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  SendMoveAction(
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
        BT::InputPort<std::string>("service_name", "The name of the service"), 
        BT::InputPort<std::string>("global_frame", "map","Global frame"),
      };
  }

protected:
  typedef std::vector<plan2_msgs::msg::Action> Actions;
  typedef rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr Client;
  using GoalHandleSendMove = rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>;
  void sendMove(Actions actions);
  Actions getMoveActions();



private:
  std::string service_name_, global_frame_;
  std::map<int, Client> clients_ptr_;
  GoalHandleSendMove::SharedPtr send_move_handler_;
  rclcpp::Node::SharedPtr node_;
  ActionStatus action_status_;
  Actions actions_;

   void goal_response_callback(const GoalHandleSendMove::SharedPtr & goal_handle);

   void feedback_callback(GoalHandleSendMove::SharedPtr, const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);

   void result_callback(const GoalHandleSendMove::WrappedResult & result);

    

}; //End Class


}  // namespace plan2_behavior_tree

#endif  // PLAN2_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_ACTION_HPP_