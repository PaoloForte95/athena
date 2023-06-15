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

#include "plan2_behavior_tree/plugins/action/dispatch_tasks_action.hpp"

namespace plan2_behavior_tree
{

DispatchTasksAction::DispatchTasksAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: ActionNodeBase(action_name, conf),
current_level_(1),max_level_(1), actions_count_(0)
{
    node_ = rclcpp::Node::make_shared("send_move_client_node");   
    IDs completed_actions;
    config().blackboard->set<IDs>("completed_actions", completed_actions);
}


inline BT::NodeStatus DispatchTasksAction::tick()
{
    setStatus(BT::NodeStatus::RUNNING);
    //Get the computed execution plan
    getInput("execution_plan", execution_plan_);
    //Get the completed actions
    getInput("completed_actions", completed_actions_);
    std::vector<plan2_msgs::msg::Action> acts = readPlan(execution_plan_);
    int exeActs = executableActions(acts);
    if(current_level_ == max_level_){
        return BT::NodeStatus::FAILURE;
    }
    if(exeActs > 0){
         return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
    
}


std::vector<plan2_msgs::msg::Action> DispatchTasksAction::readPlan(plan2_msgs::msg::Plan execution_plan_){
    Actions actions;
    for(plan2_msgs::msg::Action action : execution_plan_.actions){
        int level = 1;
        actions.push_back(action);
        for(int parID : action.parents){
            //Take the highest level from precedence constraints
            if(level <= action_levels.find(parID)->second){
                level = action_levels.find(parID)->second + 1;
                max_level_ = level;
            }
        }
        action_levels.insert(std::pair<int, int>( action.action_id, level));
    }
    
    return actions;
}

int DispatchTasksAction::executableActions(std::vector<plan2_msgs::msg::Action> actions){
    //Check which actions of the plan can be executed next
    Actions concurrent_actions, empty_set;

    std::vector<bool> levelCanBeExecuted;
    for(plan2_msgs::msg::Action action :  actions){
        int act_level = action_levels.find(action.action_id)->second;
        if(act_level == current_level_){
            bool canBeExecuted = true;
            RCLCPP_INFO(node_->get_logger(), "Current plan level %d....", current_level_);
            //Check if the actions that have a precedence constraint with this one have been completed
            for(int parID : action.parents){
                  
                auto itr = std::find(completed_actions_.begin(), completed_actions_.end(), parID);
                 //If absent, parent actions haven't been completed yet. Wa only if absent
                if (itr == completed_actions_.end()){
                    RCLCPP_INFO(node_->get_logger(), "Action %d cannot be execute because its parent %d has not been completed yet!", action.action_id, parID);
                    canBeExecuted = false;
                    break;
                }
                
            }  
            levelCanBeExecuted.push_back(canBeExecuted);
            if(canBeExecuted){
                RCLCPP_INFO(node_->get_logger(), "Sending action %d....", action.action_id);
                concurrent_actions.push_back(action);
            }else{
               
                break;
            }
        }
    }
    if(std::all_of(levelCanBeExecuted.begin(), levelCanBeExecuted.end(), [](bool v) { return v; }) && !levelCanBeExecuted.empty()){
        current_level_ ++;
        setOutput("concurrent_actions", concurrent_actions);
        actions_count_ += static_cast<int>(concurrent_actions.size());
        config().blackboard->set<int>("actions_count", actions_count_);
        return static_cast<int>(concurrent_actions.size()); 
    }
    config().blackboard->set<int>("actions_count", actions_count_);
    setOutput("concurrent_actions", empty_set);
    return 0;
    
}

plan2_msgs::msg::Action DispatchTasksAction::getAction (int ID){
    for(plan2_msgs::msg::Action act : execution_plan_.actions){
        if(act.action_id == ID){
            return act;
        }
        return plan2_msgs::msg::Action();
    }
}




} // namespace plan2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<plan2_behavior_tree::DispatchTasksAction>("DispatchTasks");
}