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

#include "athena_util/node_utils.hpp"
#include "athena_planner/state_updaters/simple_state_updater.hpp"

using std::placeholders::_1;
using rcl_interfaces::msg::ParameterType;

namespace athena_planner
{

SimpleStateUpdater::SimpleStateUpdater() {}


SimpleStateUpdater::~SimpleStateUpdater()
{
  RCLCPP_INFO(logger_, "Destroying plugin %s of type SimpleStateUpdater", name_.c_str());
}


void SimpleStateUpdater::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name)
{
  node_ = parent;
  auto node = parent.lock();
  logger_ = node->get_logger();
  name_ = name;
  
  RCLCPP_INFO(logger_, "Configuring %s of type SimpleStateUpdater", name.c_str());
  
  athena_util::declare_parameter_if_not_declared(node, name + ".type", rclcpp::ParameterValue(""));
  node->get_parameter<std::string>(name + ".type", type_);
  

  RCLCPP_INFO( logger_, "Configured plugin %s of type CostmapSelector with ", name_.c_str());

}


void SimpleStateUpdater::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s of type SimpleStateUpdater",name_.c_str());

  auto node = node_.lock();
  // Add callback for dynamic parameters
  _dyn_params_handler = node->add_on_set_parameters_callback(std::bind(&SimpleStateUpdater::dynamicParametersCallback, this, _1));
}

void SimpleStateUpdater::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type SimpleStateUpdater",
    name_.c_str());
  _dyn_params_handler.reset();
}

void SimpleStateUpdater::cleanup()
{
  RCLCPP_INFO(
    logger_, "Cleaning up plugin %s of type SimpleStateUpdater",name_.c_str());

}

athena_msgs::msg::State SimpleStateUpdater::updateState(const std::vector<athena_msgs::msg::Action> & actions, const athena_msgs::msg::State& previous_state){
    auto new_state = athena_msgs::msg::State();
    for(auto action : actions){ 
        RCLCPP_INFO(logger_, "Action name: %s",action.name.c_str());
    }
    return new_state;

}


rcl_interfaces::msg::SetParametersResult SimpleStateUpdater::dynamicParametersCallback(
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
PLUGINLIB_EXPORT_CLASS(athena_planner::SimpleStateUpdater, athena_core::StateUpdater)