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
#include <iostream>
#include <fstream>
#include <regex>
#include <vector>

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
    std::string problem_file;
    athena_msgs::msg::State inital_state;

    if(!config().blackboard->get<athena_msgs::msg::State>("previous_state", inital_state)){
      //if not present, get the initial state from the planning problem file
      getInit(inital_state);
      goal_.previous_state = inital_state;
    }
    else{
      getInput("previous_state", goal_.previous_state);
    } 
    getInput("state_updater", goal_.state_updater);
    
  
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



void UpdateStateAction::getInit(athena_msgs::msg::State &state){
  // Get the problem file from the blackboard and opent it
  RCLCPP_INFO( node_->get_logger(), "Getting the initial state from the plannign problem file."); 
  std::string problem_file;
  config().blackboard->get<std::string>("problem_file", problem_file);
  std::ifstream file(problem_file);
  if (!file) {
      RCLCPP_ERROR( node_->get_logger(), "Unable to open file."); 
      return;
  }
 
  // Regular expressions to search for
  std::regex init_pattern(R"(:init)");
  std::regex closing_parenthesis_pattern(R"(^\s*\)$)");
  std::regex whitespace_pattern(R"(^\s*)"); // Regex to match leading whitespace
 
  std::string line;
  bool capturing = false;
  std::vector<std::string> extracted_lines;
 
  while (std::getline(file, line)) {
      // Check if we found the init line
      if (std::regex_search(line, init_pattern)) {
          capturing = true; // Start capturing lines
          continue; // Skip this line
      }
      // If we are capturing, check if we should add the line
      if (capturing) {
          // Check if the current line contains only a closing parenthesis
          if (std::regex_match(line, closing_parenthesis_pattern)) {
              break; // Stop capturing if we reach a line with only ")"
          }
          line = std::regex_replace(line, whitespace_pattern, ""); 
          if (!line.empty()) {
            extracted_lines.push_back(line); // Store the line
            }
      }
  }
 
  // Output the extracted text without adding extra newline at the end
  for (size_t i = 0; i < extracted_lines.size(); ++i) {
      state.state.push_back(extracted_lines[i]);
  } 
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