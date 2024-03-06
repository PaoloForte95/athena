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

#include "athena_behavior_tree/plugins/action/dispatch_method_action.hpp"

namespace athena_behavior_tree
{

DispatchMethodsAction::DispatchMethodsAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: ActionNodeBase(action_name, conf)
{
    node_ = rclcpp::Node::make_shared("dispatch_methods_client_node");
    IDs completed_methods;
    config().blackboard->set<IDs>("completed_methods", completed_methods);
    first_time_ = true;
}


inline BT::NodeStatus DispatchMethodsAction::tick()
{
    setStatus(BT::NodeStatus::RUNNING);
    //Get the computed execution plan
    getInput("execution_plan", execution_plan_);
    //Get the completed methods
    getInput("completed_methods", completed_methods_);
    if(first_time_){
        readPlan();
    }
    bool plan_completed = true;
    for(const auto& pair : robots_methods){
        if(!pair.second.empty()){
            plan_completed = false;
            break;
        }
    }
    if(plan_completed){
        RCLCPP_INFO(node_->get_logger(), "Execution plan has been completed successfully!");
        return BT::NodeStatus::SUCCESS;
    }
    int exeActs = sendMethods();
    if(exeActs > 0){
         return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
    
}

void DispatchMethodsAction::readPlan(){
    Methods methods;
    IDs robotIDs;
    //Gettind IDs from actions
    for(athena_msgs::msg::Action action : execution_plan_.actions){
        int robotID = action.robotid;
        auto itr = std::find(robotIDs.begin(), robotIDs.end(), robotID);
        if (itr == robotIDs.end()){
            robotIDs.push_back(robotID);
            auto robot_state = "robot_" + std::to_string(robotID) + "_state";
            config().blackboard->set<std::string>(robot_state, "free");
        }
    }
    config().blackboard->set<IDs>("robot_ids", robotIDs);
    int level = 1;
    for(athena_msgs::msg::Method method : execution_plan_.methods){
        auto mutex = false;
        int subTask = method.substasks[0];
        for(athena_msgs::msg::Action action : execution_plan_.actions){
            if( action.action_id == subTask){
                RCLCPP_INFO(node_->get_logger(), "Method: %s associated to robot %d", method.name.c_str(), method.robotid);
                auto it = robots_methods.find(action.robotid);
                if (it != robots_methods.end()){
                    auto set_methods = robots_methods[action.robotid];
                    set_methods.push_back(method);
                }
                else{
                    Methods m;
                    m.push_back(method);
                    robots_methods.insert(std::pair<int, Methods>( action.robotid, m));
                }
                break;
            }
            }
        }
    first_time_ = false;
    }


int DispatchMethodsAction::sendMethods(){
    //Check which methods of the plan can be executed next
    int methods_send;
    Actions concurrent_actions;
    Methods concurrent_methods;
    for(const auto& pair : robots_methods){
        auto robot = pair.first;
        std::string robot_state;
        config().blackboard->get<std::string>("robot_" + std::to_string(robot) + "_state", robot_state);
        if(robot_state == "free"){
            bool method_can_be_executed = true;
            auto set_methods = robots_methods[robot];
            auto curr_method = set_methods[0];
            for(auto pm : curr_method.parents){
                int parent_id = pm;
                auto it_parent = std::find(completed_methods_.begin(), completed_methods_.end(), parent_id);
                if(it_parent == completed_methods_.end()){
                    method_can_be_executed = false;
                    RCLCPP_INFO(node_->get_logger(), "Method %d cannot be execute because its parent %d has not been completed yet!", curr_method.id, pm);
                    break;
                }
            }
            if(method_can_be_executed){
                RCLCPP_INFO(node_->get_logger(), "Sending method %d, %s....", curr_method.id, curr_method.name.c_str());
                concurrent_methods.push_back(curr_method);
                methods_send +=1;
                config().blackboard->set<std::string>("robot_" + std::to_string(robot) + "_state", "busy");

                for(athena_msgs::msg::Action action : execution_plan_.actions){
                    auto itr = std::find(curr_method.substasks.begin(), curr_method.substasks.end(), action.action_id);
                    if (itr != curr_method.substasks.end()) {
                        concurrent_actions.push_back(action);
                    }
                }
            }
        }
    }
    config().blackboard->set<Actions>("concurrent_actions", concurrent_actions);
    setOutput("concurrent_methods", concurrent_methods);
    return methods_send;
    
}

athena_msgs::msg::Method DispatchMethodsAction::getMethod (int ID){
    for(athena_msgs::msg::Method method : execution_plan_.methods){
        if(method.id == ID){
            return method;
        }
        return athena_msgs::msg::Method();
    }
}
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_behavior_tree::DispatchMethodsAction>("DispatchMethods");
}