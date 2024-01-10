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

#include "plan2_behavior_tree/plugins/action/dispatch_method_action.hpp"

namespace plan2_behavior_tree
{

DispatchMethodsAction::DispatchMethodsAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: ActionNodeBase(action_name, conf),
current_level_(1),max_level_(1), methods_count_(0)
{
    node_ = rclcpp::Node::make_shared("dispatch_methods_client_node");   
    IDs completed_methods;
    config().blackboard->set<IDs>("completed_methods", completed_methods);
}


inline BT::NodeStatus DispatchMethodsAction::tick()
{
    setStatus(BT::NodeStatus::RUNNING);
    //Get the computed execution plan
    getInput("execution_plan", execution_plan_);
    //Get the completed methods
    getInput("completed_methods", completed_methods_);
    if(plan_methods_.empty()){
        plan_methods_ = readPlan();
    }
    RCLCPP_INFO(node_->get_logger(), "Current plan level %d....%d", current_level_,max_level_);
    if(current_level_ >max_level_ ){
        return BT::NodeStatus::FAILURE;
    }
    int exeActs = executableMethods(plan_methods_);
    if(methods_count_ == completed_methods_.size()){
        return BT::NodeStatus::FAILURE;
    }
    if(exeActs > 0){
         return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
    
}

std::vector<plan2_msgs::msg::Method> DispatchMethodsAction::readPlan(){
    Methods methods;
    IDs robotIDs;
    //Gettind IDs from actions
    for(plan2_msgs::msg::Action action : execution_plan_.actions){
        int robotID = action.robotid;
        auto itr = std::find(robotIDs.begin(), robotIDs.end(), robotID);
        if (itr == robotIDs.end()){
            robotIDs.push_back(robotID);
        }
    }
    config().blackboard->set<IDs>("robot_ids", robotIDs);
    int level = 1;
    for(plan2_msgs::msg::Method method : execution_plan_.methods){
        RCLCPP_INFO(node_->get_logger(), "DispatchMethodsAction Method: %s", method.name.c_str());
        auto mutex = false;
        for(int parID : method.substasks){
            RCLCPP_INFO(node_->get_logger(), "DispatchMethodsAction subtask: %d", parID);
            //Take the highest level from precedence constraints
            for(plan2_msgs::msg::Method m : methods){
                auto itr = std::find(m.substasks.begin(), m.substasks.end(), parID);
                RCLCPP_INFO(node_->get_logger(), "Method %d Mutex with method: %d",method.id, m.id);
                if (itr != robotIDs.end()){
                    level += 1;
                    mutex = true;
                    break;
                }
            }
            if(mutex){
                break;
            }
        }
        methods.push_back(method);
        method_levels_.insert(std::pair<int, int>(method.id, level));
    }
    max_level_ = level;
    for(plan2_msgs::msg::Method method : execution_plan_.methods){
        int level = 1;
        RCLCPP_INFO(node_->get_logger(), "DispatchMethodsAction Method: %s", method.name.c_str());
        for(int i: method.substasks){
            RCLCPP_INFO(node_->get_logger(), "DispatchMethodsAction subtask: %d", i);
        }
        
    }
    return methods;
}

int DispatchMethodsAction::executableMethods(std::vector<plan2_msgs::msg::Method> methods){
    //Check which methods of the plan can be executed next
    Methods concurrent_methods, empty_set;
    Actions concurrent_actions;
    std::vector<bool> levelCanBeExecuted;
    for(plan2_msgs::msg::Method method : methods){
        int m_level = method_levels_.find(method.id)->second;
        if(m_level == current_level_){
            bool canBeExecuted = true;
            //Check if the actions that have a precedence constraint with this one have been completed
            for(int index = 0; index < m_level; index ++){  
                auto m = methods[index];
                if(m.id != method.id){
                auto itr = std::find(completed_methods_.begin(), completed_methods_.end(), m.id);
                 //If absent, parent actions haven't been completed yet. Wa only if absent
                    if (itr == completed_methods_.end()){
                        RCLCPP_INFO(node_->get_logger(), "Method %d cannot be execute because its parent %d has not been completed yet!", method.id, m.id);
                        canBeExecuted = false;
                        break;
                    }
                }
            }  
            levelCanBeExecuted.push_back(canBeExecuted);
            if(canBeExecuted){
                RCLCPP_INFO(node_->get_logger(), "Sending method %d, %s....", method.id, method.name.c_str());
                concurrent_methods.push_back(method);
               for(plan2_msgs::msg::Action action : execution_plan_.actions){
                    auto itr = std::find(method.substasks.begin(), method.substasks.end(), action.action_id);
                    if (itr != method.substasks.end()) {
                        concurrent_actions.push_back(action);
                    }
                }
                config().blackboard->set<Actions>("concurrent_actions", concurrent_actions);
            }else{
                break;
            }
        }
    }
    if(std::all_of(levelCanBeExecuted.begin(), levelCanBeExecuted.end(), [](bool v) { return v; }) && !levelCanBeExecuted.empty()){
        current_level_ ++;
        setOutput("concurrent_methods", concurrent_methods);
        methods_count_ += static_cast<int>(concurrent_methods.size());
        config().blackboard->set<int>("methods_count", methods_count_);
        return static_cast<int>(concurrent_methods.size()); 
    }
    
    config().blackboard->set<int>("methods_count", methods_count_);
    setOutput("concurrent_methods", empty_set);
    return 0;
    
}

plan2_msgs::msg::Method DispatchMethodsAction::getMethod (int ID){
    for(plan2_msgs::msg::Method method : execution_plan_.methods){
        if(method.id == ID){
            return method;
        }
        return plan2_msgs::msg::Method();
    }
}
} // namespace plan2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<plan2_behavior_tree::DispatchMethodsAction>("DispatchMethods");
}