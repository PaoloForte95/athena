// Copyright (c) 2024 Paolo Forte
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

#include "athena_behavior_tree/plugins/action/update_state_action.hpp"

namespace athena_behavior_tree
{

UpdateStateAction::UpdateStateAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<athena_msgs::action::UpdateState>(xml_tag_name, action_name, conf)
{  
}


void UpdateStateAction::on_tick()
{   

    config().blackboard->get<Actions>("concurrent_actions", goal_.actions);
    getInput("previous_state", goal_.previous_state);
  
}

BT::NodeStatus UpdateStateAction::on_success()
{
  setOutput("current_state", result_.result->updated_state);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus UpdateStateAction::on_aborted()
{
  setOutput("current_state", goal_.previous_state);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus UpdateStateAction::on_cancelled()
{
  setOutput("current_state", goal_.previous_state);
  return BT::NodeStatus::SUCCESS;
}


void UpdateStateAction::halt()
{
  setOutput("current_state", goal_.previous_state);
  BtActionNode::halt();
}

}



#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<athena_behavior_tree::UpdateStateAction>(
        name, "update_state", config);
    };

  factory.registerBuilder<athena_behavior_tree::UpdateStateAction>(
    "UpdateState", builder);
}