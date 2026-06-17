#ifndef ATHENA_BEHAVIOR_TREE__PLUGINS__ACTION__GENERATE_DOMAIN_FILE_ACTION_HPP_
#define ATHENA_BEHAVIOR_TREE__PLUGINS__ACTION__GENERATE_DOMAIN_FILE_ACTION_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp/action_node.h"
#include "rclcpp/rclcpp.hpp"

#include "athena_msgs/srv/get_types.hpp"
#include "athena_msgs/srv/get_predicate_list.hpp"
#include "athena_msgs/srv/get_action_list.hpp"
#include "athena_msgs/srv/generate_domain.hpp"

namespace athena_behavior_tree
{
class GenerateDomainFileAction : public BT::ActionNodeBase
{
public:
    GenerateDomainFileAction(
        const std::string & action_name, const BT::NodeConfiguration & conf);

    void halt() override {}

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts()
    {
        return
        {
            BT::OutputPort<std::string>("domain_file", "The path to the generated planning domain file"),
        };
    }

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

    rclcpp::Client<athena_msgs::srv::GetTypes>::SharedPtr get_types_client_;
    rclcpp::Client<athena_msgs::srv::GetPredicateList>::SharedPtr get_predicates_client_;
    rclcpp::Client<athena_msgs::srv::GetActionList>::SharedPtr get_actions_client_;
    rclcpp::Client<athena_msgs::srv::GenerateDomain>::SharedPtr generate_domain_client_;
};

}

#endif