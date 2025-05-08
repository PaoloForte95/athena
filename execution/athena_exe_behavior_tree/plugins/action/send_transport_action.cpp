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
#include "athena_exe_behavior_tree/plugins/action/send_transport_action.hpp"

typedef std::vector<athena_msgs::msg::Action> Actions;
typedef std::vector<int> IDs;


Eigen::Vector3d quaternionToEulerAngles(const Eigen::Quaterniond& q) {
    // Roll (x-axis rotation)
    double roll = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()), 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));

    // Pitch (y-axis rotation)
    double pitch = asin(2.0 * (q.w() * q.y() - q.z() * q.x()));

    // Yaw (z-axis rotation)
    double yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

    return Eigen::Vector3d(roll, pitch, yaw);
}


namespace athena_exe_behavior_tree
{

SendTransportAction::SendTransportAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: ActionNodeBase(action_name, conf)
{
    getInput("service_name", service_name_);
    getInput("global_frame", global_frame_);
    node_ = rclcpp::Node::make_shared("send_transport_client_node");
    auto service_name = node_->get_namespace() + service_name_;
    client_ptr_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(node_, service_name);
    waypoint_client_ = node_->create_client<location_msgs::srv::GetWaypointList>("/get_waypoint_list");
        if(waypoint_client_->wait_for_service(std::chrono::seconds(1))){
          auto request = std::make_shared<location_msgs::srv::GetWaypointList::Request>();
    auto result = waypoint_client_->async_send_request(request);
    std::vector<location_msgs::msg::Waypoint> wps;
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Got waypoint list!");
      wps = result.get()->list;
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to get waypoint list");
    }

    for (auto wp : wps){
      auto quat_w = wp.pose.orientation.w;
      auto quat_x = wp.pose.orientation.x;
      auto quat_y = wp.pose.orientation.y;
      auto quat_z = wp.pose.orientation.z;
      Eigen::Quaternion q(quat_w, quat_x,quat_y,quat_z);
    
      auto euler = quaternionToEulerAngles(q);
      Waypoint waypoint;
      waypoint.x = wp.pose.position.x;
      waypoint.y = wp.pose.position.y;
      waypoint.theta = euler[2];
      waypoints_.insert(std::make_pair(wp.name, waypoint));
      RCLCPP_INFO(node_->get_logger(),"Loading waypoint %s: (%f, %f, %f)", wp.name.c_str(), 
                  waypoint.x, waypoint.y,waypoint.theta);   
    }
    }
    else{
      Waypoint waypoint;
      waypoint.x = 0.;
      waypoint.y = 0.;
      waypoint.theta = 0.;
      waypoints_.insert(std::make_pair("dummy_waypoint", waypoint));
      RCLCPP_INFO(node_->get_logger(),"Loaded dummy waypoint");   

    }
}


inline BT::NodeStatus SendTransportAction::tick()
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


Actions SendTransportAction::getMoveActions(){
    Actions move_actions_;
    config().blackboard->get<Actions>("actions", actions_);
    for(athena_msgs::msg::Action act : actions_){
      if(act.name.find("transport") != std::string::npos){
         RCLCPP_INFO( node_->get_logger(), "Received transport action!");
         move_actions_.push_back(act);
      }
    }
    return move_actions_;
}

void SendTransportAction::sendMove(Actions actions)
  {
    using namespace std::placeholders;

    for(athena_msgs::msg::Action move_action: actions){
      std::string robot = move_action.robot;
      auto is_action_server_ready = client_ptr_->wait_for_action_server(std::chrono::seconds(5));
      auto wp2 = move_action.waypoints[move_action.waypoints.size()-1];
      auto goal_msg2 = getGoalLocation(wp2);
      while (!is_action_server_ready) { 
        RCLCPP_ERROR( node_->get_logger(), "navigate_to_pose action server is not available." " Is the initial pose set?"); 
        std::this_thread::sleep_for(std::chrono::seconds(2));
      }

      
      auto wp = move_action.waypoints[move_action.waypoints.size()-1];
      auto goal_msg = getGoalLocation(wp);
      config().blackboard->set<std::string>("dump_position", wp);
      RCLCPP_INFO(node_->get_logger(), "Sending goal");

      auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
      //send_goal_options.result_callback = [this](auto) {
      //send_move_handler_.reset();
      //};
      send_goal_options.goal_response_callback =std::bind(&SendTransportAction::goal_response_callback, this, std::placeholders::_1);
      //send_goal_options.feedback_callback = std::bind(&SendTransportAction::feedback_callback, this, std::placeholders::_1,std::placeholders::_2);
      send_goal_options.result_callback = std::bind(&SendTransportAction::result_callback, this,std::placeholders::_1);
      auto future_goal_handle = client_ptr_->async_send_goal(goal_msg, send_goal_options);
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

      rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult wrapped_result = future_result.get();
    }

}

  nav2_msgs::action::NavigateToPose::Goal SendTransportAction::getGoalLocation(std::string goal_waypoint){
      auto msg = nav2_msgs::action::NavigateToPose::Goal();
      msg.pose.header.frame_id = global_frame_;
      auto it = waypoints_.find(goal_waypoint);
      if (it == waypoints_.end()) {
         RCLCPP_ERROR(node_->get_logger()," Waypoint %s not found!", goal_waypoint.c_str());
        return msg;
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
      return msg;
  }

   void SendTransportAction::goal_response_callback(const GoalHandleSendMove::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
    }
  }


  void SendTransportAction::feedback_callback(GoalHandleSendMove::SharedPtr,const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback)
  {

    RCLCPP_INFO(node_->get_logger(), "distance %f",  feedback->distance_remaining);
  }


   void SendTransportAction::result_callback(const GoalHandleSendMove::WrappedResult & result)
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
  factory.registerNodeType<athena_exe_behavior_tree::SendTransportAction>("SendTransport");
}