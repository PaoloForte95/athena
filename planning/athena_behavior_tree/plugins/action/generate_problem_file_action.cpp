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

#include <memory>
#include <string>

#include "athena_behavior_tree/plugins/action/generate_problem_file_action.hpp"

namespace athena_behavior_tree
{

GenerateProblemFileAction::GenerateProblemFileAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: ActionNodeBase(action_name, conf)
{
    std::string image_topic;
    node_ = rclcpp::Node::make_shared("generate_planning_problem_node");   
    client_ = node_->create_client<athena_msgs::srv::GenerateProblemFile>("generate_problem_file");
    getInput("image_topic", image_topic);
    subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
            image_topic, 10,
            std::bind(&GenerateProblemFileAction::imageCallback, this, std::placeholders::_1));
}


inline BT::NodeStatus GenerateProblemFileAction::tick()
{   
    std::string format, instruction, prompt;
    setStatus(BT::NodeStatus::RUNNING);
    //Get the computed execution plan
    getInput("format", format);
    getInput("instruction", instruction);
    //Get the completed actions
    getInput("prompt", prompt);
    config().blackboard->set<std::string>("prompt", prompt);
    auto request = std::make_shared<athena_msgs::srv::GenerateProblemFile::Request>();

    request->prompt.data = prompt;
    request->instruction.data = instruction;
    request->image_file.data = "";

    try {
            // Convert ROS image message to OpenCV format
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(latest_image_, sensor_msgs::image_encodings::BGR8);
            
            // Save as PNG
            std::string filename = "captured_image.png";
            cv::imwrite(filename, cv_ptr->image);
            
            RCLCPP_INFO(node_->get_logger(), "Saved image to: %s", filename.c_str());
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if(latest_image_ == nullptr){
        RCLCPP_INFO(node_->get_logger(), "Waiting for image...");
        return BT::NodeStatus::RUNNING;
        
    }
    request->image_file.data = image_;
    // while (!client_->wait_for_service(1s)) {
    //     if (!rclcpp::ok()) {
    //         RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
    //     }
    //     RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
    // }

    auto result = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto file = result.get()->ouput_file;
        RCLCPP_INFO(node_->get_logger(), "response.ouput_file %s" , file.data.c_str());
        setOutput("problem_file", file.data);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
    
}


void GenerateProblemFileAction::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        latest_image_ = msg;
    }
} 

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_behavior_tree::GenerateProblemFileAction>("GenerateProblemFile");
}