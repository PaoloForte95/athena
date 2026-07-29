#ifndef ATHENA_BEHAVIOR_TREE__PLUGINS__ACTION__GENERATE_PROBLEM_FILE_ACTION_HPP_
#define ATHENA_BEHAVIOR_TREE__PLUGINS__ACTION__GENERATE_PROBLEM_FILE_ACTION_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/action_node.h"
#include "athena_msgs/srv/generate_problem_file.hpp"
#include "athena_msgs/srv/get_objects.hpp"

namespace athena_behavior_tree
{
/**
 * @brief A BT action node that generates a problem file based on the provided instruction
 */
class GenerateProblemFileAction : public BT::ActionNodeBase
{

public:

    /**
     * @brief A constructor for athena_behavior_tree::GenerateProblemFileAction
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
            BT::InputPort<std::string>("output_name", "problem.pddl" "The output file name"),
            BT::InputPort<std::string>("domain_file", "The path to the planning domain file"),
        };
    }
private:

    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    rclcpp::Client<athena_msgs::srv::GenerateProblemFile>::SharedPtr client_;
    rclcpp::Client<athena_msgs::srv::GetObjects>::SharedPtr get_objects_client_;
    std::string problem_instance_;
    std::string output_name_;

};

}

#endif