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
#include <fstream>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "athena_exe_behavior_tree/plugins/action/send_move_action.hpp"
#include "athena_exe_msgs/action/move_to_pose.hpp"
typedef std::vector<athena_msgs::msg::Action> Actions;
typedef std::vector<int> IDs;

namespace athena_exe_behavior_tree
{
template<typename MoveT>
SendMoveAction<MoveT>::SendMoveAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: ActionNodeBase(action_name, conf)
{
    getInput("service_name", service_name_);
    getInput("global_frame", global_frame_);
    getInput("waypoints_filename", waypoints_filename_);
    node_ = rclcpp::Node::make_shared("send_move_client_node");
    RCLCPP_ERROR(node_->get_logger(),"namespace %s", node_->get_namespace());
    auto service_name = node_->get_namespace() + service_name_;
    client_ptr_ = rclcpp_action::create_client<MoveT>(node_, service_name);
    parseWaypoints();
}


template<typename MoveT>
void SendMoveAction<MoveT>::parseWaypoints(){
    std::string file_path = ament_index_cpp::get_package_share_directory("athena_exe_launch") + "/waypoints/" + waypoints_filename_;
    std::ifstream file(file_path);
    if (!file.is_open()) {
      RCLCPP_ERROR(node_->get_logger(),"Could not open file %s", file_path.c_str() );
      return ;
    }
    RCLCPP_INFO(node_->get_logger(),"Loading waypoints from file %s", file_path.c_str());
    std::string line;
    // Parse each line for waypoints in the format "name: (x, y, theta)"
    while (std::getline(file, line)) {
      std::string name, point;
      std::string delimiter = ":";
      name = line.substr(0,line.find(delimiter));
      name.erase(std::remove_if(name.begin(), name.end(), ::isspace), name.end());
      point = line.substr(line.find(delimiter)+1);
      point.erase(std::remove_if(point.begin(), point.end(), [](char c) {
        return c == '(' || c == ')' || std::isspace(c);
      }), point.end());

      double components[3];
      size_t pos = 0;
      for (int i = 0; i < 3; ++i) {
        // Find the position of the comma from the current 'pos'
        size_t commaPos = point.find(',', pos);

        if (commaPos != std::string::npos) {
            std::string value = point.substr(pos, commaPos - pos);
            components[i] = std::stod(value);
            pos = commaPos + 1; // Move 'pos' to the character after the comma
        } else {
            // For the last component (theta), use the remaining string
            std::string value = point.substr(pos);
            components[i] = std::stod(value);
        }
    }
      Waypoint waypoint;
      waypoint.x = components[0];
      waypoint.y = components[1];
      waypoint.theta = components[2];
      waypoints_.insert(std::make_pair(name, waypoint));
      //Save the waypoints in the blackboard so that can be used by other nodes as well
      config().blackboard->set<Waypoint>(name, waypoint);
      RCLCPP_INFO(node_->get_logger(),"Loading waypoint %s: (%f, %f, %f)", name.c_str(), 
                  components[0], components[1],components[2]);   
    }
    // Close the input file
    file.close();

}

template<typename MoveT>
inline BT::NodeStatus SendMoveAction<MoveT>::tick()
{ 
    setStatus(BT::NodeStatus::RUNNING);
    
    //Get the move actions 
    Actions actions = getMoveActions();
    if(actions.empty()){
      return BT::NodeStatus::FAILURE;
    }
    sendMove(actions);
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

template<typename MoveT>
Actions SendMoveAction<MoveT>::getMoveActions(){
    Actions move_actions_;
    config().blackboard->get<Actions>("actions", actions_);
    for(athena_msgs::msg::Action act : actions_){
      if(act.name.find("move") != std::string::npos || act.name.find("drive") != std::string::npos){
         RCLCPP_INFO( node_->get_logger(), "Received moving action!");
         move_actions_.push_back(act);
      }
    }
    return move_actions_;
}




template<typename MoveT>
void SendMoveAction<MoveT>::sendMove(Actions actions)
  {
    using namespace std::placeholders;
     
    for(athena_msgs::msg::Action move_action: actions){
      int robotID = move_action.robotid;
      auto is_action_server_ready = client_ptr_->wait_for_action_server(std::chrono::seconds(5));
      auto wp = move_action.waypoints[move_action.waypoints.size()-1];
      getGoalLocation(wp);
      if (!is_action_server_ready) {
          RCLCPP_ERROR( node_->get_logger(), "navigate_to_pose action server is not available." " Is the initial pose set?"); 
          return ;
      }

      config().blackboard->set<std::string>("load_position", wp);
      RCLCPP_INFO(node_->get_logger(), "Sending goal");
      setGoalOption();
      //send_goal_options.result_callback = [this](auto) {
      //send_move_handler_.reset();
      //};
      send_goal_options_.goal_response_callback =std::bind(&SendMoveAction::goal_response_callback, this, std::placeholders::_1);
      //send_goal_options.feedback_callback = std::bind(&SendMoveAction::feedback_callback, this, std::placeholders::_1,std::placeholders::_2);
      send_goal_options_.result_callback = std::bind(&SendMoveAction::result_callback, this,std::placeholders::_1);
      auto future_goal_handle = client_ptr_->async_send_goal(goal_, send_goal_options_);
      if (rclcpp::spin_until_future_complete(node_, future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS)
      {
          RCLCPP_INFO(node_->get_logger(), "Failed sending goal");
          // failed sending the goal
          return ;
      }
      send_move_handler_ = future_goal_handle.get();
      auto future_result = client_ptr_->async_get_result(send_move_handler_);
      RCLCPP_INFO(node_->get_logger(), "Executing Operation...!");
      rclcpp::spin_until_future_complete(node_, future_result);
      GoalHandleSendMoveWrappedResult wrapped_result = future_result.get();
    }
}

template<>
  void SendMoveAction<nav2_msgs::action::NavigateToPose>::setGoalOption(){
    send_goal_options_ = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();

  }

template<>
  void SendMoveAction<athena_exe_msgs::action::MoveToPose>::setGoalOption(){
    send_goal_options_ = rclcpp_action::Client<athena_exe_msgs::action::MoveToPose>::SendGoalOptions();

  }

template<>
  void SendMoveAction<nav2_msgs::action::NavigateToPose>::getGoalLocation(std::string goal_waypoint){
      auto msg = nav2_msgs::action::NavigateToPose::Goal();
      msg.pose.header.frame_id = global_frame_;
      auto it = waypoints_.find(goal_waypoint);
      if (it == waypoints_.end()) {
         RCLCPP_ERROR(node_->get_logger()," Waypoint %s not found!", goal_waypoint.c_str());
        return ;
      }
      Waypoint loc = it->second;
      msg.pose.pose.position.x = loc.x;
      msg.pose.pose.position.y = loc.y;
      msg.pose.pose.position.z = 0.0;
      Eigen::Quaterniond quaternion = rpyToQuaternion(0, 0, loc.theta);
      RCLCPP_INFO(node_->get_logger()," Goal: %s, %f, %f, %f",goal_waypoint.c_str(),loc.x, loc.y, loc.theta);
      msg.pose.pose.orientation.x = quaternion.x();
      msg.pose.pose.orientation.y = quaternion.y();
      msg.pose.pose.orientation.z = quaternion.z();
      msg.pose.pose.orientation.w = quaternion.w();
      goal_ = msg;
  }
  
template<>
  void SendMoveAction<athena_exe_msgs::action::MoveToPose>::getGoalLocation(std::string goal_waypoint){
      auto msg = athena_exe_msgs::action::MoveToPose::Goal();
      msg.pose.header.frame_id = global_frame_;
      auto it = waypoints_.find(goal_waypoint);
      if (it == waypoints_.end()) {
         RCLCPP_ERROR(node_->get_logger()," Waypoint %s not found!", goal_waypoint.c_str());
        return ;
      }
      Waypoint loc = it->second;
      msg.pose.pose.position.x = loc.x;
      msg.pose.pose.position.y = loc.y;
      msg.pose.pose.position.z = 0.0;
      Eigen::Quaterniond quaternion = rpyToQuaternion(0, 0, loc.theta);
      RCLCPP_INFO(node_->get_logger()," Goal for Robot Arm: %s, %f, %f, %f",goal_waypoint.c_str(),loc.x, loc.y, loc.theta);
      msg.pose.pose.orientation.x = quaternion.x();
      msg.pose.pose.orientation.y = quaternion.y();
      msg.pose.pose.orientation.z = quaternion.z();
      msg.pose.pose.orientation.w = quaternion.w();
      goal_ = msg;
  }

template<typename MoveT>
   void SendMoveAction<MoveT>::goal_response_callback(const GoalHandleSendMovePtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

template<typename MoveT>
  void SendMoveAction<MoveT>::feedback_callback(GoalHandleSendMovePtr,const std::shared_ptr<const MoveFeedback> feedback)
  {

    RCLCPP_INFO(node_->get_logger(), "distance %f",  feedback->distance_remaining);
  }

template<typename MoveT>
   void SendMoveAction<MoveT>::result_callback(const GoalHandleSendMoveWrappedResult & result)
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
  factory.registerNodeType<athena_exe_behavior_tree::SendMoveAction<nav2_msgs::action::NavigateToPose>>("SendMove");
  factory.registerNodeType<athena_exe_behavior_tree::SendMoveAction<athena_exe_msgs::action::MoveToPose>>("SendArmMove");
}