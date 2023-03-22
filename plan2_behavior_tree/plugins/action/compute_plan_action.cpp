// Copyright (c) 2018 Intel Corporation
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

#include "plan2_behavior_tree/plugins/action/compute_plan_action.hpp"

namespace plan2_behavior_tree
{

ComputePlanAction::ComputePlanAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<plan2_msgs::action::ComputePlan>(xml_tag_name, action_name, conf)
{
}

void ComputePlanAction::on_tick()
{
  getInput("domain_file", goal_.planning_problem.planning_domain);
  getInput("problem_file", goal_.planning_problem.planning_problem);
  getInput("planner", goal_.planner);
  
}


BT::NodeStatus ComputePlanAction::on_success()
{
  plan2_msgs::msg::Plan empty_plan;
  setOutput("execution_plan", result_.result->execution_plan);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputePlanAction::on_aborted()
{
  plan2_msgs::msg::Plan empty_plan;
  setOutput("execution_plan", empty_plan);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputePlanAction::on_cancelled()
{
  plan2_msgs::msg::Plan empty_plan;
  setOutput("execution_plan", empty_plan);
  return BT::NodeStatus::SUCCESS;
}


void ComputePlanAction::halt()
{
  plan2_msgs::msg::Plan empty_plan;
  setOutput("execution_plan", empty_plan);
  BtActionNode::halt();
}

}  // namespace plan2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<plan2_behavior_tree::ComputePlanAction>(
        name, "compute_execution_plan", config);
    };

  factory.registerBuilder<plan2_behavior_tree::ComputePlanAction>(
    "ComputePlan", builder);
}
