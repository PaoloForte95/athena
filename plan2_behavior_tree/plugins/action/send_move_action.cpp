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

#include <filesystem>
#include "plan2_behavior_tree/plugins/action/send_move_action.hpp"


namespace plan2_behavior_tree
{

SendMoveAction::SendMoveAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: ActionNodeBase(action_name, conf),
 server_timeout_(100)
{
    getInput("service_name", service_name_);
    getInput("global_frame", global_frame_);
    
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
    client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, service_name_);

}

inline BT::NodeStatus SendMoveAction::tick()
{
setStatus(BT::NodeStatus::RUNNING);

auto result = sendMove();
if(result == rclcpp::FutureReturnCode::SUCCESS){
    return BT::NodeStatus::SUCCESS;
}
return BT::NodeStatus::FAILURE;
}


rclcpp::FutureReturnCode SendMoveAction::sendMove()
  {
    using namespace std::placeholders;

     auto is_action_server_ready =
    client_ptr_->wait_for_action_server(std::chrono::seconds(5));
  if (!is_action_server_ready) {
    RCLCPP_ERROR(
      node_->get_logger(),
      "navigate_to_pose action server is not available."
      " Is the initial pose set?");
    return rclcpp::FutureReturnCode::INTERRUPTED;
  }

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = global_frame_;
    goal_msg.pose.pose.position.x = 23.19;
    goal_msg.pose.pose.position.y = 5.99;
    goal_msg.pose.pose.position.z = 0.0;
    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = 0.0;
    goal_msg.pose.pose.orientation.w = 1.0;

    
    
    RCLCPP_INFO(node_->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback = [this](auto) {
      send_move_handler_.reset();
    };
     auto future_goal_handle = client_ptr_->async_send_goal(goal_msg, send_goal_options);
    if (rclcpp::spin_until_future_complete(node_, future_goal_handle, server_timeout_) !=
    rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Send goal call failed");
        return rclcpp::FutureReturnCode::INTERRUPTED;
    }

  // Get the goal handle and save so that we can check on completion in the timer callback
  send_move_handler_ = future_goal_handle.get();
  if (!send_move_handler_) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    return rclcpp::FutureReturnCode::INTERRUPTED;
  }
  return rclcpp::FutureReturnCode::SUCCESS;
}







}  // namespace plan2_behavior_tree


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<plan2_behavior_tree::SendMoveAction>("SendMove");
}