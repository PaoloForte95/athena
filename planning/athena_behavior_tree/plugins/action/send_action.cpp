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

#include <filesystem>
#include "athena_behavior_tree/plugins/action/send_action.hpp"

typedef std::vector<int> IDs;

namespace athena_behavior_tree
{

SendActionAction::SendActionAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: ActionNodeBase(action_name, conf)
{
    getInput("service_name", service_name_);
    getInput("robot_id", robot_id_);
    node_ = rclcpp::Node::make_shared("send_action_client_node");

    std::string service_name = "/robot"+std::to_string(robot_id_) + "/" +  service_name_;
    client_ = rclcpp_action::create_client<athena_msgs::action::ExecuteAction>(node_, service_name);
}

inline BT::NodeStatus SendActionAction::tick()
{ 
    setStatus(BT::NodeStatus::RUNNING);

    //Get the load actions 
    std::vector<athena_msgs::msg::Action> actions;
    config().blackboard->get<Actions>("concurrent_actions", actions);
    if(actions.empty()){
      return BT::NodeStatus::SUCCESS;
    }
    for (auto action: actions){
        if(action.robotid == robot_id_){
            action_ = action;
            SendAction(action);
            break;
        }
    }
    
    if(action_status_ == ActionStatus::SUCCEEDED){
      IDs completed_methods;
      config().blackboard->get<IDs>("completed_methods", completed_methods);
      std::vector<int> completed_actions;
      config().blackboard->get<IDs>("completed_actions", completed_actions);
      completed_methods.push_back(action_.action_id);
      config().blackboard->set<IDs>("completed_methods", completed_methods);
      for (auto completed_action: concurrent_actions_){
        completed_actions.push_back(completed_action.action_id);
      }
      config().blackboard->set<IDs>("completed_actions", completed_actions);
      config().blackboard->set<std::string>("robot_" + std::to_string(robot_id_) + "_state", "free");
   

      return BT::NodeStatus::SUCCESS;
    }
    else if (action_status_ == ActionStatus::FAILED){
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}


void SendActionAction::SendAction(athena_msgs::msg::Action action)
  {
    using namespace std::placeholders;


    int robotID = action.robotid;
    auto is_action_server_ready = client_->wait_for_action_server(std::chrono::seconds(5));
    if (!is_action_server_ready) { 
        RCLCPP_ERROR( node_->get_logger(), "send action server is not available."); 
        return;
    }
   
    auto goal_msg = athena_msgs::action::ExecuteAction::Goal();
    goal_msg.action = action;


    //RCLCPP_INFO(node_->get_logger(), "Sending method %s with ID %d", action_.name, action_.id);
    RCLCPP_INFO(node_->get_logger(), "Robot %d is executing action %s with ID %d...", robotID, action_.name.c_str(), action_.action_id);
    auto send_goal_options = rclcpp_action::Client<athena_msgs::action::ExecuteAction>::SendGoalOptions();
    send_goal_options.goal_response_callback =std::bind(&SendActionAction::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.result_callback = std::bind(&SendActionAction::result_callback, this,std::placeholders::_1);
    auto future_goal_handle = client_->async_send_goal(goal_msg, send_goal_options);
    if (rclcpp::spin_until_future_complete(node_, future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node_->get_logger(), "Failed sending goal"); // Failed sending the goal
        return ;
    }
    RCLCPP_INFO(node_->get_logger(), "Goal sent!");
    send_load_handler_ = future_goal_handle.get();

    auto future_result = client_->async_get_result(send_load_handler_);
    
    rclcpp::spin_until_future_complete(node_, future_result);
    rclcpp_action::ClientGoalHandle<athena_msgs::action::ExecuteAction>::WrappedResult wrapped_result = future_result.get();


}


   void SendActionAction::goal_response_callback(const GoalHandleSendLoad::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
    }
  }


  void SendActionAction::feedback_callback(GoalHandleSendLoad::SharedPtr,const std::shared_ptr<const athena_msgs::action::ExecuteAction::Feedback> feedback)
  {
    //RCLCPP_INFO(node_->get_logger(), "Executing subaction %d",  feedback->current_action_id);
  }


   void SendActionAction::result_callback(const GoalHandleSendLoad::WrappedResult & result)
  {
    sleep(8);
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "Operation completed!");
       action_status_ =  ActionStatus::SUCCEEDED;
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(node_->get_logger(), "Operation was aborted!");
        action_status_ =  ActionStatus::FAILED;
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(node_->get_logger(), "Operation was canceled!");
        action_status_ =  ActionStatus::FAILED;
        return;
      default:
        RCLCPP_ERROR(node_->get_logger(), "Unknown result code!");
        action_status_ =  ActionStatus::UNKNOWN;
        return;
    }
  }
}  // namespace athena_exe_behavior_tree


#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_behavior_tree::SendActionAction>("SendAction");
}