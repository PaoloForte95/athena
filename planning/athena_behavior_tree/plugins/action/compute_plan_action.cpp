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

#include "athena_behavior_tree/plugins/action/compute_plan_action.hpp"

namespace athena_behavior_tree
{

ComputePlanAction::ComputePlanAction(
  const std::string & xml_tag_name,
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BtActionNode<athena_msgs::action::ComputePlan>(xml_tag_name, action_name, conf)
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
  setOutput("execution_plan", result_.result->execution_plan);
  auto length = result_.result->execution_plan.actions.size();
  setOutput("plan_length",static_cast<int>(length));
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ComputePlanAction::on_aborted()
{
  athena_msgs::msg::Plan empty_plan;
  setOutput("execution_plan", empty_plan);
  setOutput("plan_length", 0);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus ComputePlanAction::on_cancelled()
{
  athena_msgs::msg::Plan empty_plan;
  setOutput("execution_plan", empty_plan);
  setOutput("plan_length", 0);
  return BT::NodeStatus::SUCCESS;
}


void ComputePlanAction::halt()
{
  athena_msgs::msg::Plan empty_plan;
  setOutput("execution_plan", empty_plan);
  setOutput("plan_length", 0);
  BtActionNode::halt();
}

}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<athena_behavior_tree::ComputePlanAction>(
        name, "compute_execution_plan", config);
    };

  factory.registerBuilder<athena_behavior_tree::ComputePlanAction>(
    "ComputePlan", builder);
}
