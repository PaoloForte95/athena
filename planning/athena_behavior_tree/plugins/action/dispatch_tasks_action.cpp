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

#include "athena_behavior_tree/plugins/action/dispatch_tasks_action.hpp"

namespace athena_behavior_tree
{

DispatchTasksAction::DispatchTasksAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: ActionNodeBase(action_name, conf),
current_level_(1),max_level_(1), actions_count_(0)
{
    node_ = rclcpp::Node::make_shared("dispatch_tasks_client_node");   
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
    if(plan_actions_.empty()){
        plan_actions_ = readPlan();
    }
    RCLCPP_INFO(node_->get_logger(), "Current plan level %d....%d", current_level_,max_level_);
    if(current_level_ >max_level_ ){
        return BT::NodeStatus::SUCCESS;
    }
    int exeActs = executableActions(plan_actions_);
    if(actions_count_ == completed_actions_.size()){
        return BT::NodeStatus::SUCCESS;
    }
    if(exeActs > 0){
         return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
    
}

std::vector<athena_msgs::msg::Action> DispatchTasksAction::readPlan(){
    Actions actions;
     std::vector<std::string> robotIDs;
    for(athena_msgs::msg::Action action : execution_plan_.actions){
        int level = 1;
        std::string robotID = action.robot;
        auto itr = std::find(robotIDs.begin(), robotIDs.end(), robotID);
        if (itr == robotIDs.end()){
            robotIDs.push_back(robotID);
        }
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
    config().blackboard->set< std::vector<std::string>>("robot_ids", robotIDs);
    
    return actions;
}

int DispatchTasksAction::executableActions(std::vector<athena_msgs::msg::Action> actions){
    //Check which actions of the plan can be executed next
    Actions concurrent_actions, empty_set;

    std::vector<bool> levelCanBeExecuted;
    int prev_action_blocked = -1;
    for(athena_msgs::msg::Action action :  actions){
        int act_level = action_levels.find(action.action_id)->second;
        if(act_level == current_level_){
            bool canBeExecuted = true;
            //Check if the actions that have a precedence constraint with this one have been completed
            for(int parID : action.parents){   
                auto itr = std::find(completed_actions_.begin(), completed_actions_.end(), parID);
                 //If absent, parent actions haven't been completed yet. Wa only if absent
                if (itr == completed_actions_.end()){
                    if(prev_action_blocked != action.action_id){
                        prev_action_blocked = action.action_id;
                        RCLCPP_INFO_ONCE(node_->get_logger(), "Action %d cannot be execute because its parent %d has not been completed yet!", action.action_id, parID);
                    }
                    
                    canBeExecuted = false;
                    break;
                }
            }  
            levelCanBeExecuted.push_back(canBeExecuted);
            if(canBeExecuted){
                RCLCPP_INFO(node_->get_logger(), "Sending action %d, %s....", action.action_id, action.name.c_str());
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

athena_msgs::msg::Action DispatchTasksAction::getAction (int ID){
    for(athena_msgs::msg::Action act : execution_plan_.actions){
        if(act.action_id == ID){
            return act;
        }
        return athena_msgs::msg::Action();
    }
}

} // namespace athena_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_behavior_tree::DispatchTasksAction>("DispatchTasks");
}