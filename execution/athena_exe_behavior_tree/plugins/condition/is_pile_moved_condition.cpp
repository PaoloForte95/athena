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
  node_ = rclcpp::Node::make_shared("check_material_amount_client_node");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  getInput<std::string>("criteria", criteria_);
  getInput<std::string>("global_frame", global_frame_);
  getInput<std::string>("scan_topic", scan_topic_);
  getInput<std::string>("image_topic", image_topic_);
  getInput<int>("ticks", ticks_);
  getInput<double>("threshold", threshold_);
 
  scan_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(scan_topic_,
  rclcpp::SystemDefaultsQoS(),
  std::bind(&IsPileMovedCondition::lidarSensorCallback, this, std::placeholders::_1));


   image_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(image_topic_,
   rclcpp::SystemDefaultsQoS(),
   std::bind(&IsPileMovedCondition::cameraSensorCallback, this, std::placeholders::_1));

   material_amount_client_ =  node_->create_client<athena_exe_msgs::srv::GetMaterialAmount>("get_material_amount");
}

BT::NodeStatus IsPileMovedCondition::tick()
{
  current_tick_ += 1;
  RCLCPP_INFO(node_->get_logger(), "Checking material amount using %s data", criteria_.c_str());
  if(criteria_ == "Amount"){
    std::string material;
    config().blackboard->get<std::string>("material_loaded", material);
    auto request = std::make_shared<athena_exe_msgs::srv::GetMaterialAmount::Request>();
    request->pile_id = material;

    auto result = material_amount_client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
      double current_amount = result.get()->amount;
      if(current_amount > threshold_){
        RCLCPP_INFO(node_->get_logger(), "Progress Material %s moving: %f... %f", material.c_str(), current_amount, threshold_);
        return BT::NodeStatus::FAILURE;
      }
      RCLCPP_INFO(node_->get_logger(), "Material %s has been moved!", material.c_str());
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Failed to get material %s amount",material.c_str());
    }
  }
  
  else if(criteria_ == "LidarScan"){
    RCLCPP_INFO(node_->get_logger(),"Using lidar for checking the pile" );
    tf2::Transform tf_transform;
    getTransform(
        data_->header.frame_id, global_frame_,
        tf2::durationFromSec(0.5), tf_buffer_, tf_transform);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*data_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*data_, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*data_, "z");
      
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      // Transform point coordinates from source frame -> to base frame
      tf2::Vector3 p_v3_s(*iter_x, *iter_y, *iter_z);
      tf2::Vector3 p_v3_b = tf_transform * p_v3_s;
      if (p_v3_b.z() >= threshold_ ) {
        RCLCPP_INFO(node_->get_logger(), "Z value exceeds threshold: %f", *iter_z);
        return BT::NodeStatus::FAILURE;
        }
    }
    return BT::NodeStatus::SUCCESS;
  }else if(criteria_ == "Camera"){
    RCLCPP_INFO(node_->get_logger(),"Using lidar for checking the pile" );
    tf2::Transform tf_transform;
    getTransform(
        data_->header.frame_id, global_frame_,
        tf2::durationFromSec(0.5), tf_buffer_, tf_transform);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*data_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*data_, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*data_, "z");
      
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      // Transform point coordinates from source frame -> to base frame
      tf2::Vector3 p_v3_s(*iter_x, *iter_y, *iter_z);
      tf2::Vector3 p_v3_b = tf_transform * p_v3_s;
      if (p_v3_b.z() >= threshold_ ) {
        RCLCPP_INFO(node_->get_logger(), "Z value exceeds threshold: %f", *iter_z);
        return BT::NodeStatus::FAILURE;
        }
    }
    return BT::NodeStatus::SUCCESS;

  }
  else if(criteria_ == "NumberCycles"){
        RCLCPP_INFO(node_->get_logger(),"Using estimated number of cycles for checking the pile...");
        if(current_tick_ < ticks_){
          RCLCPP_INFO(node_->get_logger(),"Cycle %d/%d... Moving Pile", current_tick_, ticks_);
            return BT::NodeStatus::FAILURE;
        }
        RCLCPP_INFO(node_->get_logger(),"Cycle %d/%d... Move Piled completed", current_tick_, ticks_);
        current_tick_ = 0;
        return BT::NodeStatus::SUCCESS;
  }
  else{
     RCLCPP_INFO(node_->get_logger(),"Criteria %s not supported. Choose between: LidarScan, Camera, Amount or NumberCycles", criteria_.c_str() );
     BT::NodeStatus::IDLE;
  }   
}

  void IsPileMovedCondition::lidarSensorCallback(const sensor_msgs::msg::PointCloud2::SharedPtr data)
  {
    data_ = data;
  }

  void IsPileMovedCondition::cameraSensorCallback(const sensor_msgs::msg::PointCloud2::SharedPtr data)
  {
    depth_image_ = data;
  }


}// namespace




#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_exe_behavior_tree::IsPileMovedCondition>("IsPileMoved");
}