#include "athena_behavior_tree/plugins/action/send_move_action.hpp"

typedef std::vector<athena_msgs::msg::Action> Actions;
typedef std::vector<int> IDs;

namespace athena_behavior_tree
{

SendMoveAction::SendMoveAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: ActionNodeBase(action_name, conf),
  action_status_(ActionStatus::UNKNOWN)
{
  getInput("service_name", service_name_);
  getInput("robot_id", robot_id_);

  node_ = rclcpp::Node::make_shared("send_move_client_node");
  const std::string full_action_name = robot_id_ + "/" + service_name_;
  client_ptr_ = rclcpp_action::create_client<MoveToPose>(node_, full_action_name);
}

BT::NodeStatus SendMoveAction::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  Actions actions = getMoveActions();
  if (actions.empty()) {
    return BT::NodeStatus::RUNNING;
  }

  if (!sendMove(actions)) {
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

Actions SendMoveAction::getMoveActions()
{
  Actions move_actions;
  config().blackboard->get<Actions>("concurrent_actions", actions_);

  for (const athena_msgs::msg::Action & act : actions_) {
    if (act.name.find("move")  != std::string::npos ||
        act.name.find("go")    != std::string::npos ||
        act.name.find("drive") != std::string::npos)
    {
      if (act.robot == robot_id_) {
        RCLCPP_INFO(node_->get_logger(), "Received '%s' action for %s", act.name.c_str(), robot_id_.c_str());
        move_actions.push_back(act);
      }
    }
  }
  return move_actions;
}

bool SendMoveAction::sendMove(Actions actions)
{
  using namespace std::placeholders;

  if (!client_ptr_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_->get_logger(), "'%s' action server is not available.", service_name_.c_str());
    return false;
  }

  for (const athena_msgs::msg::Action & move_action : actions) {
    if (move_action.waypoints.empty()) {
      RCLCPP_ERROR(node_->get_logger(), "Action '%s' has no waypoints, skipping.", move_action.name.c_str());
      return false;
    }

    const std::string & wp_name = move_action.waypoints.back();
    config().blackboard->set<std::string>("load_position", wp_name);

    auto goal = buildGoal(wp_name);

    auto send_goal_options = rclcpp_action::Client<MoveToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&SendMoveAction::goal_response_callback, this, _1);
    send_goal_options.result_callback =
      std::bind(&SendMoveAction::result_callback, this, _1);

    RCLCPP_INFO(node_->get_logger(), "Sending MoveToPose goal for waypoint '%s'", wp_name.c_str());
    auto future_goal_handle = client_ptr_->async_send_goal(goal, send_goal_options);

    if (rclcpp::spin_until_future_complete(node_, future_goal_handle) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to send goal for waypoint '%s'", wp_name.c_str());
      return false;
    }

    goal_handle_ = future_goal_handle.get();
    if (!goal_handle_) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the action server.");
      return false;
    }

    auto future_result = client_ptr_->async_get_result(goal_handle_);
    RCLCPP_INFO(node_->get_logger(), "Waiting for result...");
    rclcpp::spin_until_future_complete(node_, future_result);
  }

  return true;
}

SendMoveAction::MoveToPose::Goal SendMoveAction::buildGoal(const std::string & wp_name)
{
  MoveToPose::Goal goal;

  auto it = waypoints_.find(wp_name);
  if (it == waypoints_.end()) {
    RCLCPP_WARN(node_->get_logger(), "Waypoint '%s' not found, using zero pose.", wp_name.c_str());
    return goal;
  }

  const Waypoint & wp = it->second;
  Eigen::Quaterniond q = rpyToQuaternion(0.0, 0.0, wp.theta);

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = node_->now();
  goal.target_pose.pose.position.x = wp.x;
  goal.target_pose.pose.position.y = wp.y;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation.x = q.x();
  goal.target_pose.pose.orientation.y = q.y();
  goal.target_pose.pose.orientation.z = q.z();
  goal.target_pose.pose.orientation.w = q.w();

  RCLCPP_INFO(node_->get_logger(), "Goal: %s -> (%.2f, %.2f, %.2f)",
              wp_name.c_str(), wp.x, wp.y, wp.theta);
  return goal;
}

void SendMoveAction::goal_response_callback(const GoalHandleMoveToPose::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the server.");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Goal accepted, waiting for result.");
  }
}

void SendMoveAction::result_callback(const GoalHandleMoveToPose::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(node_->get_logger(), "MoveToPose succeeded!");
      action_status_ = ActionStatus::SUCCEEDED;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "MoveToPose was aborted.");
      action_status_ = ActionStatus::FAILED;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(node_->get_logger(), "MoveToPose was canceled.");
      action_status_ = ActionStatus::FAILED;
      break;
    default:
      RCLCPP_ERROR(node_->get_logger(), "Unknown result code.");
      action_status_ = ActionStatus::UNKNOWN;
      break;
  }
}

}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_behavior_tree::SendMoveAction>("SendMove");
}