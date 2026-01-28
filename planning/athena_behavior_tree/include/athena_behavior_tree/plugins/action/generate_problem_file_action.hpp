// Copyright (c) 2025 Paolo Forte
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

#ifndef ATHENA_BEHAVIOR_TREE__PLUGINS__ACTION__GENERATE_PROBLEM_FILE_ACTION_HPP_
#define ATHENA_BEHAVIOR_TREE__PLUGINS__ACTION__GENERATE_PROBLEM_FILE_ACTION_HPP_

#include <string>
#include <memory>

#include "athena_behavior_tree/bt_action_node.hpp"
#include "athena_msgs/srv/generate_problem_file.hpp"
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
namespace athena_behavior_tree
{
/**
 * @brief A athena_behavior_tree::BtActionNode class that wraps athena_behavior_tree::action::DispatchTasksAction
 */
class GenerateProblemFileAction : public BT::ActionNodeBase
{

public:

    /**
     * @brief A constructor for athena_behavior_tree::DispatchTasksAction
     * @param action_name Action name this node creates a client for
     * @param conf BT node configuration
     */
    GenerateProblemFileAction(
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
            BT::OutputPort<std::string>("problem_file","The path to the object location file"),
            BT::InputPort<std::string>("format", "PDDL", "The format of the file to generate, either PDDL or HDDL"), 
            BT::InputPort<std::string>("instruction", "The instruction to the VLM"), 
            BT::InputPort<std::string>("prompt", "The promt to the VLM"),
            BT::InputPort<std::string>("image_topic", "/camera/image_raw", "The topic to get the image from")
        };
    }
private:
    
     void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);

    rclcpp::Node::SharedPtr node_;
    rclcpp::Client<athena_msgs::srv::GenerateProblemFile>::SharedPtr client_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    sensor_msgs::msg::Image::SharedPtr latest_image_;
    std::string problem_instance_;
    std::string image_;



}; //Class end

}  

#endif