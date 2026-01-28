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

#include "athena_planner/planners/metric_ff.hpp"

#include "athena_protobuf/execution_plan.hpp"
#include "athena_protobuf/action.hpp"
#include "athena_protobuf/executionplan.pb.h"


using std::placeholders::_1;
using rcl_interfaces::msg::ParameterType;

namespace athena_planner
{

MetricFF::MetricFF() {}


MetricFF::~MetricFF()
{
  RCLCPP_INFO(logger_, "Destroying plugin %s of type MetricFF", name_.c_str());
}


void MetricFF::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name)
{
  node_ = parent;
  auto node = parent.lock();
  logger_ = node->get_logger();
  name_ = name;
  
  RCLCPP_INFO(logger_, "Configuring %s of type MetricFF", name.c_str());

}


void MetricFF::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s of type MetricFF",name_.c_str());

  auto node = node_.lock();
  // Add callback for dynamic parameters
  _dyn_params_handler = node->add_on_set_parameters_callback(std::bind(&MetricFF::dynamicParametersCallback, this, _1));
}

void MetricFF::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type MetricFF",
    name_.c_str());
  _dyn_params_handler.reset();
}

void MetricFF::cleanup()
{
  RCLCPP_INFO(
    logger_, "Cleaning up plugin %s of type MetricFF",name_.c_str());

}

athena_msgs::msg::Plan MetricFF::computeExecutionPlan(const std::string & domain, const std::string & problem){

athena_msgs::msg::Plan execution_plan;
int status = system(("java -jar src/athena/planning/athena_planner/Planners/task_planner.jar metricff " + 
domain + " " + 
problem + " " 
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



bool MetricFF::validateDomain(const std::string & domain){


}


bool MetricFF::validateProblem(const std::string & problem){

}


rcl_interfaces::msg::SetParametersResult MetricFF::dynamicParametersCallback(
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
PLUGINLIB_EXPORT_CLASS(athena_planner::MetricFF, athena_core::Planner)