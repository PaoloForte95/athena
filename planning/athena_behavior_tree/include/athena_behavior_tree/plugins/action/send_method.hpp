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


#ifndef ATHENA_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_METHOD_ACTION_HPP_
#define ATHENA_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_METHOD_ACTION_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "athena_msgs/action/execute_method.hpp"
#include "athena_behavior_tree/bt_action_node.hpp"
#include "athena_msgs/msg/action.hpp"
typedef std::vector<athena_msgs::msg::Action> Actions;
namespace athena_behavior_tree
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

class SendMethodAction : public BT::ActionNodeBase
{
public:
  /**
   * @brief A constructor for athena_behavior_tree::SendLoadAction
   * @param action_name Action name this node creates a client for
   * @param conf BT node configuration
   */
  SendMethodAction(
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
         BT::InputPort<std::string>("robot", "The robot to be associated with this action"), 

      };
  }

protected:
  typedef rclcpp_action::Client<athena_msgs::action::ExecuteMethod>::SharedPtr Client;
  using GoalHandleSendLoad = rclcpp_action::ClientGoalHandle<athena_msgs::action::ExecuteMethod>;
  void sendMethod(athena_msgs::msg::Method method);



private:
  std::string service_name_, global_frame_;
  GoalHandleSendLoad::SharedPtr send_load_handler_;
  rclcpp::Node::SharedPtr node_;
  ActionStatus action_status_;
  athena_msgs::msg::Method method_;
  Actions concurrent_actions_;
  std::string robot_;
  Client client_;

   void goal_response_callback(const GoalHandleSendLoad::SharedPtr & goal_handle);

   void feedback_callback(GoalHandleSendLoad::SharedPtr, const std::shared_ptr<const athena_msgs::action::ExecuteMethod::Feedback> feedback);

   void result_callback(const GoalHandleSendLoad::WrappedResult & result);

    

}; //End Class


}  //end namespace

#endif  