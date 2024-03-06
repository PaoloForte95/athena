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
#include "athena_exe_behavior_tree/plugins/action/send_load_action.hpp"

typedef std::vector<athena_msgs::msg::Action> Actions;
typedef std::vector<int> IDs;

namespace athena_exe_behavior_tree
{

SendLoadAction::SendLoadAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: ActionNodeBase(action_name, conf)
{
    getInput("service_name", service_name_);
    getInput("global_frame", global_frame_);
    node_ = rclcpp::Node::make_shared("send_load_client_node");
    auto service_name = node_->get_namespace() + service_name_;
    client_ptr_ = rclcpp_action::create_client<athena_exe_msgs::action::BucketCommand>(node_, service_name);
}

inline BT::NodeStatus SendLoadAction::tick()
{ 
    setStatus(BT::NodeStatus::RUNNING);


    //Get the load actions 
    Actions actions = getLoadActions();
    if(actions.empty()){
      return BT::NodeStatus::FAILURE;
    }
    sendLoad(actions);
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


Actions  SendLoadAction::getLoadActions(){
    Actions load_actions_;
    config().blackboard->get<Actions>("actions", actions_);
    for(athena_msgs::msg::Action act : actions_){
      if(act.name.find("load") != std::string::npos || act.name.find("pick") != std::string::npos || act.name.find("unstack") != std::string::npos){
         RCLCPP_INFO( node_->get_logger(), "Action: %s, Received loading action!", act.name.c_str());
         load_actions_.push_back(act);
      }
    }
    return load_actions_;
}

void SendLoadAction::sendLoad(Actions actions)
  {
    using namespace std::placeholders;

    for(athena_msgs::msg::Action load_action: actions){
      int robotID = load_action.robotid;
      auto is_action_server_ready = client_ptr_->wait_for_action_server(std::chrono::seconds(5));
      if (!is_action_server_ready) { 
          RCLCPP_ERROR( node_->get_logger(), "load action server is not available."); 
          return;
      }
      auto goal_msg = athena_exe_msgs::action::BucketCommand::Goal();
      goal_msg.target_boom = 0.4;
      goal_msg.target_bucket = 0.4;
      goal_msg.material_id = load_action.material;
      RCLCPP_INFO(node_->get_logger(), "Sending load");
      auto send_goal_options = rclcpp_action::Client<athena_exe_msgs::action::BucketCommand>::SendGoalOptions();
      send_goal_options.goal_response_callback =std::bind(&SendLoadAction::goal_response_callback, this, std::placeholders::_1);
      send_goal_options.result_callback = std::bind(&SendLoadAction::result_callback, this,std::placeholders::_1);
      auto future_goal_handle = client_ptr_->async_send_goal(goal_msg, send_goal_options);
      if (rclcpp::spin_until_future_complete(node_, future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS)
      {
          RCLCPP_INFO(node_->get_logger(), "Failed sending load"); // Failed sending the goal
          return ;
      }
      send_load_handler_ = future_goal_handle.get();

      auto future_result = client_ptr_->async_get_result(send_load_handler_);
      RCLCPP_INFO(node_->get_logger(), "Executing loading for robot %d...!", robotID);
      rclcpp::spin_until_future_complete(node_, future_result);
      rclcpp_action::ClientGoalHandle<athena_exe_msgs::action::BucketCommand>::WrappedResult wrapped_result = future_result.get();
    }

}


   void SendLoadAction::goal_response_callback(const GoalHandleSendLoad::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
    }
  }


  void SendLoadAction::feedback_callback(GoalHandleSendLoad::SharedPtr,const std::shared_ptr<const athena_exe_msgs::action::BucketCommand::Feedback> feedback)
  {
    RCLCPP_INFO(node_->get_logger(), "Boom Position %f",  feedback->position_boom);
    RCLCPP_INFO(node_->get_logger(), "Bucket Position %f",  feedback->position_bucket);
  }


   void SendLoadAction::result_callback(const GoalHandleSendLoad::WrappedResult & result)
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


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_exe_behavior_tree::SendLoadAction>("SendLoad");
}