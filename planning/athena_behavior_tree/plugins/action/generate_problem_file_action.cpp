#include <memory>
#include <string>

#include "athena_behavior_tree/plugins/action/generate_problem_file_action.hpp"
#include "behaviortree_cpp/bt_factory.h"

namespace athena_behavior_tree
{

GenerateProblemFileAction::GenerateProblemFileAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: ActionNodeBase(action_name, conf)
{
    getInput("output_name", output_name_);
    node_ = rclcpp::Node::make_shared("generate_planning_problem_node");
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
    rclcpp::SubscriptionOptions sub_option;
    sub_option.callback_group = callback_group_;

    client_ = node_->create_client<athena_msgs::srv::GenerateProblemFile>("generate_problem_file");
    get_objects_client_ = node_->create_client<athena_msgs::srv::GetObjects>("get_objects");
}

BT::NodeStatus GenerateProblemFileAction::tick()
{
    setStatus(BT::NodeStatus::RUNNING);

    std::string instruction, domain_file;
    config().blackboard->get<std::string>("instruction", instruction);
    RCLCPP_INFO(node_->get_logger(), "instruction: %s", instruction.c_str());

    auto objects_request = std::make_shared<athena_msgs::srv::GetObjects::Request>();
    auto objects_result = get_objects_client_->async_send_request(objects_request);
    if (rclcpp::spin_until_future_complete(node_, objects_result) != rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_ERROR(node_->get_logger(), "Failed to call get_objects service");
        return BT::NodeStatus::FAILURE;
    }

    auto objects_response = objects_result.get();

    auto request = std::make_shared<athena_msgs::srv::GenerateProblemFile::Request>();
    request->instruction = instruction;
    request->objects = objects_response->objects;
    request->init = objects_response->init;
    getInput("domain_file", domain_file);
    request->domain = domain_file;

    auto result = client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto file = result.get()->problem_file;
        RCLCPP_INFO(node_->get_logger(), "response.problem_file %s", file.data.c_str());
        setOutput("problem_file", file.data);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
}

}  // namespace athena_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_behavior_tree::GenerateProblemFileAction>("GenerateProblemFile");
}