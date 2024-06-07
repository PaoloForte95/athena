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


#ifndef ATHENA_CE_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_DUMP_ACTION_HPP_
#define ATHENA_CE_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_DUMP_ACTION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "athena_exe_msgs/action/load_or_dump.hpp"
#include "athena_behavior_tree/bt_action_node.hpp"
#include "athena_msgs/msg/action.hpp"

namespace athena_exe_behavior_tree
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

class SendDumpAction : public BT::ActionNodeBase
{
public:
  /**
   * @brief A constructor for athena_behavior_tree::SendDumpAction
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  SendDumpAction(
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
  typedef std::vector<athena_msgs::msg::Action> Actions;
  typedef rclcpp_action::Client<athena_exe_msgs::action::LoadOrDump>::SharedPtr Client;
  using GoalHandleSendDump = rclcpp_action::ClientGoalHandle<athena_exe_msgs::action::LoadOrDump>;
  void sendDump(Actions actions);
  Actions getDumpActions();



private:
  std::string service_name_, global_frame_;
  Client client_ptr_;
  GoalHandleSendDump::SharedPtr send_dump_handler_;
  rclcpp::Node::SharedPtr node_;
  ActionStatus action_status_;
  Actions actions_;

   void goal_response_callback(const GoalHandleSendDump::SharedPtr & goal_handle);

   void feedback_callback(GoalHandleSendDump::SharedPtr, const std::shared_ptr<const athena_exe_msgs::action::LoadOrDump::Feedback> feedback);

   void result_callback(const GoalHandleSendDump::WrappedResult & result);

}; //End Class

}  // namespace athena_behavior_tree

#endif  