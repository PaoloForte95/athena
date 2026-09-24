#include "athena_behavior_tree/plugins/action/send_load_action.hpp"

typedef std::vector<athena_msgs::msg::Action> Actions;
typedef std::vector<int> IDs;

namespace athena_behavior_tree
{

SendLoadAction::SendLoadAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: ActionNodeBase(action_name, conf),
  action_status_(ActionStatus::UNKNOWN)
{
  getInput("service_name", service_name_);
  getInput("robot_id", robot_id_);

  node_ = rclcpp::Node::make_shared("send_load_client_node");
  const std::string full_action_name = robot_id_ + "/" + service_name_;
  client_ptr_ = rclcpp_action::create_client<standard_msgs::action::Load>(node_, full_action_name);
}

BT::NodeStatus SendLoadAction::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  Actions actions = getLoadActions();
  if (actions.empty()) {
    return BT::NodeStatus::RUNNING;
  }

  if (!sendLoad(actions)) {
    return BT::NodeStatus::FAILURE;
  }

  if (action_status_ == ActionStatus::SUCCEEDED) {
    IDs completed_actions;
    config().blackboard->get<IDs>("completed_actions", completed_actions);
    for (const athena_msgs::msg::Action & act : actions) {
      completed_actions.push_back(act.action_id);
    }
    config().blackboard->set<IDs>("completed_actions", completed_actions);
    return BT::NodeStatus::SUCCESS;
  } else if (action_status_ == ActionStatus::FAILED) {
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::RUNNING;
}

Actions SendLoadAction::getLoadActions()
{
  Actions load_actions;
  config().blackboard->get<Actions>("concurrent_actions", actions_);

  for (const athena_msgs::msg::Action & act : actions_) {
    if (act.name.find("load")    != std::string::npos ||
        act.name.find("pick")    != std::string::npos ||
        act.name.find("unstack") != std::string::npos)
    {
      if (act.robot == robot_id_) {
        RCLCPP_INFO(node_->get_logger(), "Received '%s' action for %s", act.name.c_str(), robot_id_.c_str());
        load_actions.push_back(act);
      }
    }
  }
  return load_actions;
}

bool SendLoadAction::sendLoad(Actions actions)
{
  using namespace std::placeholders;

  if (!client_ptr_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_->get_logger(), "'%s' action server is not available.", service_name_.c_str());
    return false;
  }

  for (const athena_msgs::msg::Action & load_action : actions) {
    std::string location = load_action.waypoints[0];
    auto goal_msg = standard_msgs::action::Load::Goal();
    goal_msg.location = location;
    goal_msg.target = load_action.object;
    goal_msg.amount = 1;

    RCLCPP_INFO(node_->get_logger(), "Loading object '%s' at '%s'", load_action.object.c_str(), location.c_str());
    config().blackboard->set<std::string>(robot_id_ + "_object_loaded", load_action.object);

    auto send_goal_options = rclcpp_action::Client<standard_msgs::action::Load>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&SendLoadAction::goal_response_callback, this, _1);
    send_goal_options.result_callback =
      std::bind(&SendLoadAction::result_callback, this, _1);

    RCLCPP_INFO(node_->get_logger(), "Sending Load goal for robot '%s'", robot_id_.c_str());
    auto future_goal_handle = client_ptr_->async_send_goal(goal_msg, send_goal_options);

    if (rclcpp::spin_until_future_complete(node_, future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to send goal for loading '%s'", load_action.object.c_str());
      return false;
    }

    send_load_handler_ = future_goal_handle.get();
    if (!send_load_handler_) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the action server.");
      return false;
    }

    auto future_result = client_ptr_->async_get_result(send_load_handler_);
    RCLCPP_INFO(node_->get_logger(), "Waiting for result...");
    rclcpp::spin_until_future_complete(node_, future_result);
  }

  return true;
}

void SendLoadAction::goal_response_callback(const GoalHandleSendLoad::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the server.");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Goal accepted, waiting for result.");
  }
}

void SendLoadAction::result_callback(const GoalHandleSendLoad::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "Load succeeded!");
      config().blackboard->set<std::string>(robot_id_ + "_state", "free");
      action_status_ = ActionStatus::SUCCEEDED;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "Load was aborted.");
      config().blackboard->set<std::string>(robot_id_ + "_state", "failure");
      action_status_ = ActionStatus::FAILED;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node_->get_logger(), "Load was canceled.");
      config().blackboard->set<std::string>(robot_id_ + "_state", "free");
      action_status_ = ActionStatus::FAILED;
      break;
    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code.");
      config().blackboard->set<std::string>(robot_id_ + "_state", "failure");
      action_status_ = ActionStatus::UNKNOWN;
      break;
  }
}

}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_behavior_tree::SendLoadAction>("SendLoad");
}