// Copyright (c) 2026 Paolo Forte
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


#include "athena_util/node_utils.hpp"

#include "athena_planner/planners/fast_downward.hpp"

#include "athena_protobuf/execution_plan.hpp"
#include "athena_protobuf/action.hpp"
#include "athena_protobuf/executionplan.pb.h"


using std::placeholders::_1;
using rcl_interfaces::msg::ParameterType;

namespace athena_planner
{

FastDownward::FastDownward() {}


FastDownward::~FastDownward()
{
  RCLCPP_INFO(logger_, "Destroying plugin %s of type FastDownward", name_.c_str());
}


void FastDownward::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name)
{
  node_ = parent;
  auto node = parent.lock();
  logger_ = node->get_logger();
  name_ = name;

  RCLCPP_INFO(logger_, "Configuring %s of type FastDownward", name.c_str());

  if (!node->has_parameter(name_ + ".search")) {
    node->declare_parameter(name_ + ".search", rclcpp::ParameterValue(search_));
  }
  node->get_parameter(name_ + ".search", search_);

  RCLCPP_INFO(logger_, "Fast Downward search configuration: %s", search_.c_str());
}


void FastDownward::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s of type FastDownward", name_.c_str());

  auto node = node_.lock();
  // Add callback for dynamic parameters
  _dyn_params_handler = node->add_on_set_parameters_callback(std::bind(&FastDownward::dynamicParametersCallback, this, _1));
}

void FastDownward::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type FastDownward",
    name_.c_str());
  _dyn_params_handler.reset();
}

void FastDownward::cleanup()
{
  RCLCPP_INFO(
    logger_, "Cleaning up plugin %s of type FastDownward", name_.c_str());

}

athena_msgs::msg::Plan FastDownward::computeExecutionPlan(const std::string & domain, const std::string & problem){

  athena_msgs::msg::Plan execution_plan;

  std::string search;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    search = search_;
  }

  int status = system(("java -jar src/athena/planning/athena_planner/Planners/task_planner.jar fd " +
    domain + " " +
    problem + " \"" +
    search + "\""
  ).c_str());

  if (status != 0) {
    RCLCPP_ERROR(logger_, "Cannot compute the execution plan!");
    return execution_plan;
  }


  athena_protobuf::Plan plan;
  athena::ProtoExecutionPlan execution_proto_plan = plan.ParseFile(proto_filename_);
  execution_plan.actions = plan.GetActions(execution_proto_plan);
  return execution_plan;
}



bool FastDownward::validateDomain(const std::string & domain){
  (void)domain;
  return true;
}


bool FastDownward::validateProblem(const std::string & problem){
  (void)problem;
  return true;
}


rcl_interfaces::msg::SetParametersResult FastDownward::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_STRING) {
      if (name == name_ + ".search") {
        search_ = parameter.as_string();
        RCLCPP_INFO(logger_, "Fast Downward search configuration changed to: %s", search_.c_str());
      }
    }
  }

  result.successful = true;
  return result;
}


} //namespace athena_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(athena_planner::FastDownward, athena_planning_core::Planner)
