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
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/convert.h"
#include <chrono>
#include <memory>
#include <string>

namespace athena_exe_behavior_tree
{

bool getTransform(
  const std::string & source_frame_id,
  const std::string & target_frame_id,
  const tf2::Duration & transform_tolerance,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  tf2::Transform & tf2_transform)
{
  geometry_msgs::msg::TransformStamped transform;
  tf2_transform.setIdentity();  // initialize by identical transform

  if (source_frame_id == target_frame_id) {
    // We are already in required frame
    return true;
  }

  try {
    // Obtaining the transform to get data from source to target frame
    transform = tf_buffer->lookupTransform(
      target_frame_id, source_frame_id,
      tf2::TimePointZero, transform_tolerance);
  } catch (tf2::TransformException & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("getTransform"),
      "Failed to get \"%s\"->\"%s\" frame transform: %s",
      source_frame_id.c_str(), target_frame_id.c_str(), e.what());
    return false;
  }

  // Convert TransformStamped to TF2 transform
  tf2::fromMsg(transform.transform, tf2_transform);
  return true;
}



IsPileMovedCondition::IsPileMovedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf), current_tick_(0)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  getInput<std::string>("criteria", criteria_);
  getInput<std::string>("image_topic", image_topic_);
  getInput<int>("ticks", ticks_);
  getInput<double>("desired_amount", desired_amount_);
  material_amount_client_ =  node_->create_client<material_handler_msgs::srv::GetMaterialAmount>("get_material_amount");
}

BT::NodeStatus IsPileMovedCondition::tick()
{
  current_tick_ += 1;
  RCLCPP_INFO(node_->get_logger(), "Moving material using %s criteria", criteria_.c_str());
  if(criteria_ == "Amount"){
    std::string material, location;
    config().blackboard->get<std::string>("material_loaded", material);
    config().blackboard->get<std::string>("dump_position", location);
    //Get the info from the simulator
    auto request = std::make_shared<material_handler_msgs::srv::GetMaterialAmount::Request>();
    request->pile_id = material;
    request->pile_location = location;

    auto result = material_amount_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
    double current_amount = result.get()->amount;
      if(current_amount < desired_amount_){
        RCLCPP_INFO(node_->get_logger(), "Progress Material %s moving: %f... %f", material.c_str(), current_amount, desired_amount_);
        return BT::NodeStatus::FAILURE;
      }
      RCLCPP_INFO(node_->get_logger(), "Material %s has been moved!", material.c_str());
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get material %s amount",material.c_str());
    }
  }
  else if(criteria_ == "Camera"){
    RCLCPP_INFO(node_->get_logger(),"Using camera for checking the pile. Making the call to the VLM model..." );
    
    return BT::NodeStatus::SUCCESS;

  }
  else if(criteria_ == "NumberCycles"){
        //FIXME Add the function to estimate automatically the number of load cycles
        RCLCPP_INFO(node_->get_logger(),"Using estimated number of cycles for checking the pile...");
        if(current_tick_ < ticks_){
          RCLCPP_INFO(node_->get_logger(),"Cycle %d/%d... Moving Pile", current_tick_, ticks_);
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(node_->get_logger(),"Cycle %d/%d... Move Pile completed", current_tick_, ticks_);
        current_tick_ = 0;
        return BT::NodeStatus::SUCCESS;
  }
  else{
     RCLCPP_INFO(node_->get_logger(),"Criteria %s not supported. Choose between: LidarScan, Camera, Amount or NumberCycles", criteria_.c_str() );
     BT::NodeStatus::IDLE;
  }   
}


}// namespace




#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_exe_behavior_tree::IsPileMovedCondition>("IsPileMoved");
}