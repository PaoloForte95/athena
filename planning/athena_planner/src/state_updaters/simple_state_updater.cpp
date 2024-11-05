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
#include <regex>
#include "athena_util/node_utils.hpp"
#include "athena_planner/state_updaters/simple_state_updater.hpp"

using std::placeholders::_1;
using rcl_interfaces::msg::ParameterType;

namespace athena_planner
{

std::string transformState(const std::string& state) {
    // Regular expression to check for negative state format
    std::regex negative_pattern(R"(\(not\s*\((.+?)\)\))");

    std::smatch match;
    if (std::regex_match(state, match, negative_pattern)) {
        // If it's a negative state (e.g., "(not (at red_plate wp1s))"),
        // return the positive part inside "(not (...))"
        return "(" + match[1].str() + ")";
    } else {
        // If it's a positive state (e.g., "(at red_plate wp1s)"),
        // return it in the negative format "(not (at red_plate wp1s))"
        return "(not " + state + ")";
    }
}

void addState(std::vector<std::string>& states, const std::string& new_state) {
    // Transform the new state to its opposite form
    std::string opposite_state = transformState(new_state);

    // Check if the opposite state exists in the vector
    auto it = std::find(states.begin(), states.end(), opposite_state);
    if (it != states.end()) {
        // Remove the opposite state if it exists
        states.erase(it);
        //std::cout << "Removed opposite state: " << opposite_state << std::endl;
    }

    // Check if the state itself is already in the vector before adding
    if (std::find(states.begin(), states.end(), new_state) == states.end()) {
        states.push_back(new_state);
        //std::cout << "Added state: " << new_state << std::endl;
    } else {
        //std::cout << "State already exists: " << new_state << std::endl;
    }
}

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
    auto state = previous_state.state;
    for(auto action : actions){ 
      for(auto effect : action.effects){
        addState(state, effect);
        //RCLCPP_INFO(logger_, "Transformed Action effect: %s", changed_effect.c_str());
      }
    }
    // for (auto s : state){
    //   RCLCPP_INFO(logger_, "Final state: %s", s.c_str());
    // }
    new_state.state = state;
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