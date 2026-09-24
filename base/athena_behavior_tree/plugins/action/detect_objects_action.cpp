// Copyright (c) 2025 Paolo Forte
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
#include <fstream>
#include <sstream>
#include <yaml-cpp/yaml.h>
#include "athena_behavior_tree/plugins/action/detect_objects_action.hpp"

namespace athena_behavior_tree
{

DetectObjectsAction::DetectObjectsAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: ActionNodeBase(action_name, conf)
{
    node_ = rclcpp::Node::make_shared("dispatch_tasks_client_node");   
    client_ = node_->create_client<athena_msgs::srv::DetectObjects>("detect_objects");
}


inline BT::NodeStatus DetectObjectsAction::tick()
{
    setStatus(BT::NodeStatus::RUNNING);
    //Get the computed PDDL problem instance
    getInput("problem_instance", problem_instance_);
    std::ifstream file(problem_instance_);
    std::stringstream buffer;
    buffer << file.rdbuf(); // Read the whole file into the buffer
    std::string file_contents = buffer.str(); // Convert buffer to string

    auto request = std::make_shared<athena_msgs::srv::DetectObjects::Request>();

    request->problem_file.data = file_contents;

    // while (!client_->wait_for_service(1s)) {
    //     if (!rclcpp::ok()) {
    //         RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
    //     }
    //     RCLCPP_INFO(node_->get_logger(), "service not available, waiting again...");
    // }

    auto result = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::SUCCESS;
    
}

}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_behavior_tree::DetectObjectsAction>("DetectObjects");
}