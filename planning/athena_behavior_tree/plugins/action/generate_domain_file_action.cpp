#include <chrono>
#include <memory>
#include <string>

#include "athena_behavior_tree/plugins/action/generate_domain_file_action.hpp"


template<typename ServiceT>
typename ServiceT::Response::SharedPtr call_service(
  rclcpp::executors::SingleThreadedExecutor & executor,
  const typename rclcpp::Client<ServiceT>::SharedPtr & client,
  const typename ServiceT::Request::SharedPtr & request)
{
    if (!client->wait_for_service(std::chrono::seconds(5))) {
        return nullptr;
    }
    auto future = client->async_send_request(request);
    if (executor.spin_until_future_complete(future) != rclcpp::FutureReturnCode::SUCCESS) {
        return nullptr;
    }
    return future.get();
}


namespace athena_behavior_tree
{

GenerateDomainFileAction::GenerateDomainFileAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(action_name, conf)
{
    node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(
      callback_group_, node_->get_node_base_interface());

    get_types_client_ = node_->create_client<athena_msgs::srv::GetTypes>(
      "get_types", rmw_qos_profile_services_default, callback_group_);
    get_predicates_client_ = node_->create_client<athena_msgs::srv::GetPredicateList>(
      "get_predicate_list", rmw_qos_profile_services_default, callback_group_);
    get_actions_client_ = node_->create_client<athena_msgs::srv::GetActionList>(
      "get_action_list", rmw_qos_profile_services_default, callback_group_);
    generate_domain_client_ = node_->create_client<athena_msgs::srv::GenerateDomain>(
      "generate_domain", rmw_qos_profile_services_default, callback_group_);
}

BT::NodeStatus GenerateDomainFileAction::tick()
{

    auto types_req = std::make_shared<athena_msgs::srv::GetTypes::Request>();
    auto types_res = call_service<athena_msgs::srv::GetTypes>(callback_group_executor_, get_types_client_, types_req);
    if (!types_res) {
        RCLCPP_ERROR(node_->get_logger(), "get_types service call failed");
        return BT::NodeStatus::FAILURE;
    }

    auto pred_req = std::make_shared<athena_msgs::srv::GetPredicateList::Request>();
    auto pred_res = call_service<athena_msgs::srv::GetPredicateList>(callback_group_executor_, get_predicates_client_, pred_req);
    if (!pred_res) {
        RCLCPP_ERROR(node_->get_logger(), "get_predicate_list service call failed");
        return BT::NodeStatus::FAILURE;
    }

    auto act_req = std::make_shared<athena_msgs::srv::GetActionList::Request>();
    auto act_res = call_service<athena_msgs::srv::GetActionList>(callback_group_executor_, get_actions_client_, act_req);
    if (!act_res) {
        RCLCPP_ERROR(node_->get_logger(), "get_action_list service call failed");
        return BT::NodeStatus::FAILURE;
    }
    
    auto gen_req = std::make_shared<athena_msgs::srv::GenerateDomain::Request>();
    gen_req->types = types_res->types;
    gen_req->predicates = pred_res->predicates;
    gen_req->actions = act_res->actions;

    auto gen_res = call_service<athena_msgs::srv::GenerateDomain>(callback_group_executor_, generate_domain_client_, gen_req);
    if (!gen_res || !gen_res->success) {
        RCLCPP_ERROR(node_->get_logger(), "generate_domain service call failed");
        return BT::NodeStatus::FAILURE;
    }

    setOutput("domain_file", gen_res->path);
    RCLCPP_INFO(node_->get_logger(), "Domain saved: %s", gen_res->path.c_str());
    return BT::NodeStatus::SUCCESS;
}

}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_behavior_tree::GenerateDomainFileAction>("GenerateDomainFile");
}