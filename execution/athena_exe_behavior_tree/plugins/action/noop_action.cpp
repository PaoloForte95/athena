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
#include "athena_exe_behavior_tree/plugins/action/noop_action.hpp"

typedef std::vector<athena_msgs::msg::Action> Actions;
typedef std::vector<int> IDs;

namespace athena_exe_behavior_tree
{

NoopAction::NoopAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: ActionNodeBase(action_name, conf)
{
    getInput("service_name", service_name_);
    getInput("waiting_time", waiting_time_);
    node_ = rclcpp::Node::make_shared("send_noop_client_node");
}

inline BT::NodeStatus NoopAction::tick()
{ 
    setStatus(BT::NodeStatus::RUNNING);
    if(clients_ptr_.empty()){
      std::vector<std::string> robotIDs;
      config().blackboard->get<std::vector<std::string>>("robot_ids", robotIDs);
      for(std::string robotID : robotIDs){
        std::string service_name = robotID + "/" + service_name_;
        auto client_ptr = rclcpp_action::create_client<nav2_msgs::action::Wait>(node_, service_name);
        clients_ptr_.insert(std::pair<std::string, Client>(robotID, client_ptr));
      }
    }

    //Get the move actions 
    Actions actions = getNoopActions();
    //No current actions, return success
    if(actions.empty()){
      return BT::NodeStatus::SUCCESS;
    }
    sendNoop(actions);
    if(action_status_ == ActionStatus::SUCCEEDED){
      IDs completed_actions;
      config().blackboard->get<IDs>("completed_actions", completed_actions);
      for(athena_msgs::msg::Action act: actions){
        completed_actions.push_back(act.action_id);
      }
      config().blackboard->set<IDs>("completed_actions", completed_actions);
        return BT::NodeStatus::SUCCESS;
    }
    else if (action_status_ == ActionStatus::FAILED){
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
}


Actions  NoopAction::getNoopActions(){
    Actions noop_actions_;
    config().blackboard->get<Actions>("concurrent_actions", actions_);
    for(athena_msgs::msg::Action act : actions_){
      if(act.name.find("noop") != std::string::npos){
         RCLCPP_INFO( node_->get_logger(), "Received noop action! Just doing nothing");
         noop_actions_.push_back(act);
      }
    }
    return noop_actions_;
}

void NoopAction::sendNoop(Actions actions)
  {
    using namespace std::placeholders;
     
    for(athena_msgs::msg::Action noop_action: actions){
        std::string robot = noop_action.robot;

        auto client_ptr = clients_ptr_.find(robot)->second;
        
        client_ptr->wait_for_action_server(std::chrono::seconds(5));
       
        auto is_action_server_ready = client_ptr->wait_for_action_server(std::chrono::seconds(5));
        
        while (!is_action_server_ready) { 
        RCLCPP_ERROR( node_->get_logger(), "wait action server is not available." " Is the initial pose set?"); 
        std::this_thread::sleep_for(std::chrono::seconds(2));
      }
       
        auto goal_msg = nav2_msgs::action::Wait::Goal();
        goal_msg.time = rclcpp::Duration(waiting_time_,0);

        RCLCPP_INFO(node_->get_logger(), "Sending waiting...");

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::Wait>::SendGoalOptions();
        //send_goal_options.result_callback = [this](auto) {
        //send_move_handler_.reset();
        //};
        send_goal_options.goal_response_callback =std::bind(&NoopAction::goal_response_callback, this, std::placeholders::_1);
        //send_goal_options.feedback_callback = std::bind(&SendMoveAction::feedback_callback, this, std::placeholders::_1,std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&NoopAction::result_callback, this,std::placeholders::_1);
        auto future_goal_handle = client_ptr->async_send_goal(goal_msg, send_goal_options);
        if (rclcpp::spin_until_future_complete(node_, future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(node_->get_logger(), "Failed sending goal");
            // failed sending the goal
            return ;
        }
        send_noop_handler_ = future_goal_handle.get();

        auto future_result = client_ptr->async_get_result(send_noop_handler_);
        RCLCPP_INFO(node_->get_logger(), "Waiting since of no Operation...!");
        rclcpp::spin_until_future_complete(node_, future_result);

        rclcpp_action::ClientGoalHandle<nav2_msgs::action::Wait>::WrappedResult wrapped_result = future_result.get();
        }

}


   void NoopAction::goal_response_callback(const GoalHandleSendWait::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
    }
  }


  void NoopAction::feedback_callback(GoalHandleSendWait::SharedPtr,const std::shared_ptr<const nav2_msgs::action::Wait::Feedback> feedback)
  {

    RCLCPP_INFO(node_->get_logger(), "Still need to wait %f seconds",  feedback->time_left);
  }


   void NoopAction::result_callback(const GoalHandleSendWait::WrappedResult & result)
  {
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
  factory.registerNodeType<athena_exe_behavior_tree::NoopAction>("Noop");
}