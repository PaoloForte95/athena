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


#include "athena_exe_behavior_tree/plugins/condition/is_pile_moved_condition.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace athena_exe_behavior_tree
{

IsPileMovedCondition::IsPileMovedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf), current_tick_(0)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  auto test = rclcpp::Node::make_shared("costmap_node");
  getInput<std::string>("criteria", criteria_);
  getInput<std::string>("global_frame", global_frame_);
  getInput<std::string>("costmap_topic", costmap_topic_);
  getInput<int>("ticks", ticks_);
  getInput<double>("threshold", threshold_);
 
  costmap_client_ = node_->create_client<nav2_msgs::srv::GetCostmap>(costmap_topic_);
}

BT::NodeStatus IsPileMovedCondition::tick()
{
    current_tick_ += 1;
    if(criteria_ == "LidarScan"){
      RCLCPP_INFO(node_->get_logger(),"Using lidar for checking the pile" );
      auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
      while (!costmap_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
      }
      auto result = costmap_client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node_, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_INFO(node_->get_logger(), "Success:");
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call service %s", costmap_topic_);
      }
      auto data = result.get()->map.data;
      for(auto value : data){
        if(value >threshold_){
          return BT::NodeStatus::FAILURE;
        }
      }
      return BT::NodeStatus::SUCCESS;

  }else if(criteria_ == "NumberCycles"){
        RCLCPP_INFO(node_->get_logger(),"Using estimated number of cycles for checking the pile...");
        if(current_tick_ < ticks_){
          RCLCPP_INFO(node_->get_logger(),"Cycle %d/%d... Moving Pile", current_tick_, ticks_);
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(node_->get_logger(),"Cycle %d/%d... Move Piled completed", current_tick_, ticks_);
        return BT::NodeStatus::SUCCESS;
  }
  else{
     RCLCPP_INFO(node_->get_logger(),"Criteria %s not supported. Choose between: LidarScan or NumberCycles", criteria_.c_str() );
     BT::NodeStatus::IDLE;
  }
    
}
}  // namespace

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_exe_behavior_tree::IsPileMovedCondition>("IsPileMoved");
}