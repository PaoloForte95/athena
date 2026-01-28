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
    auto all_methods = execution_plan_.methods;
    //Get the completed methods
    getInput("completed_methods", completed_methods_);
    if(first_time_){
        readPlan();
        start_time_ = node_->get_clock()->now();
    }
    bool plan_completed = true;
    for(auto comp_method : all_methods){     
        if(completed_methods_.size() == all_methods.size()){
            break;
        }
        auto itr = std::find(completed_methods_.begin(), completed_methods_.end(), int(comp_method.id));
        if (itr == completed_methods_.end()){
            plan_completed = false;
            break;
        }
    }
    // for(const auto& pair : robots_methods){
    //     RCLCPP_INFO(node_->get_logger(), "robot/method: %s %d" , pair.first.c_str(), pair.second[0].id);
    //     if(!pair.second.empty()){
    //         plan_completed = false;
    //         break;
    //     }
    // }
    std::vector<std::string> robotIDs;
    config().blackboard->get<std::vector<std::string>>("robot_ids", robotIDs);

    for(std::string robot: robotIDs){
        std::string robot_state;
        config().blackboard->get<std::string>(robot + "_state", robot_state);
         if(robot_state == "busy"){
            plan_completed = false;
            break;
         }
    }
    if(plan_completed){
        RCLCPP_INFO(node_->get_logger(), "Execution plan has been completed successfully!");
        rclcpp::Time end_time = node_->get_clock()->now();
        rclcpp::Duration diff = end_time - start_time_;
        RCLCPP_INFO(node_->get_logger(), "Execution Time: %f seconds", diff.seconds());
        return BT::NodeStatus::SUCCESS;
    }
    sendMethods();
    return BT::NodeStatus::FAILURE;
    
}

void DispatchMethodsAction::readPlan(){
    Methods methods;
    std::vector<std::string> robotIDs;
    //Gettind IDs from actions
    for(athena_msgs::msg::Action action : execution_plan_.actions){
        std::string robot = action.robot;
        auto itr = std::find(robotIDs.begin(), robotIDs.end(), robot);
        if (itr == robotIDs.end()){
            robotIDs.push_back(robot);
            auto robot_state = robot + "_state";
            config().blackboard->set<std::string>(robot_state, "free");
        }
    }
    config().blackboard->set< std::vector<std::string>>("robot_ids", robotIDs);
    int level = 1;
    for(athena_msgs::msg::Method method : execution_plan_.methods){
        auto mutex = false;
        int subTask = method.substasks[0];
        for(athena_msgs::msg::Action action : execution_plan_.actions){
            if( action.action_id == subTask){
                RCLCPP_INFO(node_->get_logger(), "Method: (%s, %d) associated to robot %s", method.name.c_str(), method.id, method.robot.c_str());
                auto it = robots_methods.find(action.robot);
                Methods set_methods;
                if (it != robots_methods.end()){
                    set_methods = robots_methods[action.robot];
                    set_methods.push_back(method);
                      robots_methods[action.robot] = set_methods;
                     
                }
                else{
                    set_methods.push_back(method);
                    robots_methods.insert(std::pair<std::string, Methods>( action.robot, set_methods));
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
        config().blackboard->get<std::string>(robot + "_state", robot_state);
        RCLCPP_INFO(node_->get_logger(), "robot %s state %s!", robot.c_str(), robot_state.c_str());
        
        if(robot_state == "free"){
            bool method_can_be_executed = true;
            auto set_methods = robots_methods[robot];
            if (set_methods.empty()){
                continue;
            }
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
                robots_methods[robot].erase(robots_methods[robot].begin());
                methods_send +=1;
                config().blackboard->set<std::string>(robot + "_state", "busy");

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
    return concurrent_methods.size();
    
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

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_behavior_tree::DispatchMethodsAction>("DispatchMethods");
}