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

#include <chrono>
#include <string>
#include <memory>
#include <cmath>

#include "athena_behavior_tree/plugins/decorator/updated_prompt_controller.hpp"

namespace athena_behavior_tree
{

UpdatedPromptController::UpdatedPromptController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  first_time_(false)
{
  
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
   first_time_ = true;
}

inline BT::NodeStatus UpdatedPromptController::tick()
{
    std::string current_prompt;
    config().blackboard->get<std::string>("prompt", current_prompt);
    setStatus(BT::NodeStatus::RUNNING);
    BT::NodeStatus child_state = BT::NodeStatus::IDLE;
    if(first_time_ || current_prompt != previous_prompt_){
        RCLCPP_INFO(node_->get_logger(), "Prompt changed, executing child node!");
        first_time_ = false;
        previous_prompt_ = current_prompt;
        child_state = child_node_->executeTick();
    }

    switch (child_state) {
        case BT::NodeStatus::RUNNING:
            return BT::NodeStatus::RUNNING;

        case BT::NodeStatus::SUCCESS:
            return BT::NodeStatus::SUCCESS;

        case BT::NodeStatus::FAILURE:
        default:
            return BT::NodeStatus::FAILURE;
    }
  return status();
}

} 

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_behavior_tree::UpdatedPromptController>("UpdatedPromptController");
}

