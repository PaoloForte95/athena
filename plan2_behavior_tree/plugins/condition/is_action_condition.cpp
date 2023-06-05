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

#include "plan2_behavior_tree/plugins/condition/is_action_condition.hpp"
#include <chrono>
#include <memory>
#include <string>

namespace plan2_behavior_tree
{

IsActionEqualCondition::IsActionEqualCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  getInput<std::string>("action_name", action_name_);
  first_iteration_ = true;
 
}

BT::NodeStatus IsActionEqualCondition::tick()
{
  if(!first_iteration_){
    config().blackboard->get<std::string>("action", actual_action_);
  }else {
     first_iteration_ = false;
  }
  if (action_name_.find("move") == std::string::npos )
  {
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
  
}

}  // namespace plan2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<plan2_behavior_tree::IsActionEqualCondition>("IsAction");
}
