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

#ifndef ATHENA_BEHAVIOR_TREE__PLUGINS__ACTION__DETECT_OBJECT_HPP_
#define ATHENA_BEHAVIOR_TREE__PLUGINS__ACTION__DETECT_OBJECT_HPP_

#include <string>
#include <memory>

#include "athena_behavior_tree/bt_action_node.hpp"
#include "athena_msgs/srv/detect_objects.hpp"
namespace athena_behavior_tree
{
/**
 * @brief A athena_behavior_tree::BtActionNode class that wraps athena_behavior_tree::action::DispatchTasksAction
 */
class DetectObjectsAction : public BT::ActionNodeBase
{
    
public:

    /**
     * @brief A constructor for athena_behavior_tree::DispatchTasksAction
     * @param action_name Action name this node creates a client for
     * @param conf BT node configuration
     */
    DetectObjectsAction(
        const std::string & action_name,
        const BT::NodeConfiguration & conf);

    /**
     * @brief The other (optional) override required by a BT action.
     */
    void halt() override {}

    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific ports
     */
    static BT::PortsList providedPorts()
    {
        return 
        {   
            BT::InputPort<std::string>("image_path"," ",  "The path to the saved image"), 
            BT::InputPort<std::string>("depth_image_path"," ", "The location to the saved depth image"), 
            BT::InputPort<std::string>("problem_instance", "The path to the generated PDDL problem instance image"), 
        };
    }
private:
  
  
  rclcpp::Node::SharedPtr node_;
  
  std::string problem_instance_;
  rclcpp::Client<athena_msgs::srv::DetectObjects>::SharedPtr client_;



}; //Class end

}  

#endif