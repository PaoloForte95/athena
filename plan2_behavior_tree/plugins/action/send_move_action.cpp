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

typedef std::vector<plan2_msgs::msg::Action> Actions;
typedef std::vector<int> IDs;

namespace plan2_behavior_tree
{

SendMoveAction::SendMoveAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: ActionNodeBase(action_name, conf)
{
    getInput("service_name", service_name_);
    getInput("global_frame", global_frame_);
    node_ = rclcpp::Node::make_shared("send_move_client_node");
}

inline BT::NodeStatus SendMoveAction::tick()
{ 
    setStatus(BT::NodeStatus::RUNNING);
    if(clients_ptr_.empty()){
      IDs robotIDs;
      
      config().blackboard->get<IDs>("robot_ids", robotIDs);
      for(int robotID : robotIDs){
        std::string service_name = "/robot"+std::to_string(robotID) + service_name_;
        
        auto client_ptr = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, service_name);
        clients_ptr_.insert(std::pair<int, Client>(robotID, client_ptr));
      }
    }

    //Get the move actions 
    Actions actions = getMoveActions();
    sendMove(actions);
    if(action_status_ == ActionStatus::SUCCEEDED){
      IDs completed_actions;
      config().blackboard->get<IDs>("completed_actions", completed_actions);
      for(plan2_msgs::msg::Action act: actions){
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


Actions  SendMoveAction::getMoveActions(){
    Actions move_actions_;
    config().blackboard->get<Actions>("concurrent_actions", actions_);
    for(plan2_msgs::msg::Action act : actions_){
      if(act.name.find("move") != std::string::npos || act.name.find("drive") != std::string::npos){
         RCLCPP_INFO( node_->get_logger(), "Received moving action!");
         move_actions_.push_back(act);
      }
    }
    return move_actions_;
}

void SendMoveAction::sendMove(Actions actions)
  {
    using namespace std::placeholders;

    for(plan2_msgs::msg::Action move_action: actions){
      int robotID = move_action.robotid;
      auto client_ptr = clients_ptr_.find(robotID)->second;
      auto is_action_server_ready = client_ptr->wait_for_action_server(std::chrono::seconds(5));
      if (!is_action_server_ready) { 
          RCLCPP_ERROR( node_->get_logger(), "navigate_to_pose action server is not available." " Is the initial pose set?"); 
          return ;
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
      //send_goal_options.result_callback = [this](auto) {
      //send_move_handler_.reset();
      //};
      send_goal_options.goal_response_callback =std::bind(&SendMoveAction::goal_response_callback, this, std::placeholders::_1);
      //send_goal_options.feedback_callback = std::bind(&SendMoveAction::feedback_callback, this, std::placeholders::_1,std::placeholders::_2);
      send_goal_options.result_callback = std::bind(&SendMoveAction::result_callback, this,std::placeholders::_1);
      auto future_goal_handle = client_ptr->async_send_goal(goal_msg, send_goal_options);
      if (rclcpp::spin_until_future_complete(node_, future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS)
      {
          RCLCPP_INFO(node_->get_logger(), "Failed sending goal");
          // failed sending the goal
          return ;
      }
      send_move_handler_ = future_goal_handle.get();

      auto future_result = client_ptr->async_get_result(send_move_handler_);
      RCLCPP_INFO(node_->get_logger(), "Executing Operation...!");
      rclcpp::spin_until_future_complete(node_, future_result);

      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult wrapped_result = future_result.get();
    }

}


   void SendMoveAction::goal_response_callback(const GoalHandleSendMove::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
    }
  }


  void SendMoveAction::feedback_callback(GoalHandleSendMove::SharedPtr,const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
  {

    RCLCPP_INFO(node_->get_logger(), "distance %f",  feedback->distance_remaining);
  }


   void SendMoveAction::result_callback(const GoalHandleSendMove::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_ERROR(node_->get_logger(), "Operation completed!");
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

}  // namespace plan2_behavior_tree


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<plan2_behavior_tree::SendMoveAction>("SendMove");
}