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

#include "athena_util/node_utils.hpp"

#include "athena_planner/planners/lpg.hpp"

#include "athena_protobuf/execution_plan.hpp"
#include "athena_protobuf/action.hpp"
#include "athena_protobuf/executionplan.pb.h"

using std::placeholders::_1;
using rcl_interfaces::msg::ParameterType;

namespace athena_planner
{

LPG::LPG() {}


LPG::~LPG()
{
  RCLCPP_INFO(logger_, "Destroying plugin %s of type LPGTD", name_.c_str());
}


void LPG::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name)
{
  node_ = parent;
  auto node = parent.lock();
  logger_ = node->get_logger();
  name_ = name;
  
  RCLCPP_INFO(logger_, "Configuring %s of type LPGTD", name.c_str());
  
    athena_util::declare_parameter_if_not_declared(node, name + ".problem_type", rclcpp::ParameterValue(""));
  node->get_parameter<std::string>(name + ".problem_type", problem_type_);
  

  athena_util::declare_parameter_if_not_declared(node, name + ".output_name", rclcpp::ParameterValue("plan.pddl"));
  node->get_parameter<std::string>(name + ".output_name", output_filename_);

  athena_util::declare_parameter_if_not_declared(node, name + ".proto_filename", rclcpp::ParameterValue("ExePlan.data"));
  node->get_parameter<std::string>(name + ".proto_filename", proto_filename_);

  athena_util::declare_parameter_if_not_declared(node, name + ".definition.robot", rclcpp::ParameterValue(""));
  node->get_parameter<std::string>(name + ".definition.robot", robot_definition_);

  athena_util::declare_parameter_if_not_declared(node, name + ".definition.location", rclcpp::ParameterValue(""));
  node->get_parameter<std::string>(name + ".definition.location", location_definition_);

  athena_util::declare_parameter_if_not_declared(node, name + ".plan_type", rclcpp::ParameterValue("TOTAL_ORDERED"));
  node->get_parameter<std::string>(name + ".plan_type", plan_type_);

  RCLCPP_INFO( logger_, "Configured plugin %s of type CostmapSelector with ", name_.c_str());

}


void LPG::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s of type LPGTD",name_.c_str());

  auto node = node_.lock();
  // Add callback for dynamic parameters
  _dyn_params_handler = node->add_on_set_parameters_callback(std::bind(&LPG::dynamicParametersCallback, this, _1));
}

void LPG::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type LPGTD",
    name_.c_str());
  _dyn_params_handler.reset();
}

void LPG::cleanup()
{
  RCLCPP_INFO(
    logger_, "Cleaning up plugin %s of type LPGTD",name_.c_str());

}

athena_msgs::msg::Plan LPG::computeExecutionPlan(const std::string & domain, const std::string & problem){

athena_msgs::msg::Plan execution_plan;
RCLCPP_INFO( logger_, "Test: %s, %s, %s ", output_filename_.c_str(), plan_type_.c_str(), robot_definition_.c_str());
int status = system(("java -jar src/athena/planning/athena_planner/Planners/task_planner.jar lpg " + 
problem_type_ + " " +
domain + " " + 
problem + " " + 
plan_type_+ " " +
output_filename_ + " " +
robot_definition_ + " " +
location_definition_
).c_str());



if (status == -1) {
    RCLCPP_ERROR(logger_, "Cannot compute the execution plan!");
    return execution_plan;
}


  athena_protobuf::Plan plan;
  athena::ProtoExecutionPlan execution_proto_plan = plan.ParseFile(proto_filename_);
  execution_plan.actions = plan.GetActions(execution_proto_plan);
  return execution_plan;
}



bool LPG::validateDomain(const std::string & domain){}


bool LPG::validateProblem(const std::string & problem){}


rcl_interfaces::msg::SetParametersResult LPG::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  bool reinit_downsampler = false;


  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {


    } else if (type == ParameterType::PARAMETER_BOOL) {

    }
    else if (type == ParameterType::PARAMETER_INTEGER){

    }
    else if (type == ParameterType::PARAMETER_STRING){

    
    }
  }

  result.successful = true;
  return result;
}


} //namespace athena_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(athena_planner::LPG, athena_core::Planner)