#include <vector>
#include <string>
#include <set>
#include <memory>
#include <limits>
#include "athena_bt_planner/planners/generate_actions.hpp"
#include "std_msgs/msg/string.hpp"
#include "athena_msgs/msg/planning_problem.hpp"

namespace athena_bt_planner
{

bool TaskPlanner::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  start_time_ = rclcpp::Time(0);
  auto node = parent_node.lock();
  if (!node) {
    return false;
  }

  if (!node->has_parameter("behavior_tree_blackboard_id")) {
    node->declare_parameter("behavior_tree_blackboard_id", std::string("behavior_tree"));
  }
  behavior_tree_blackboard_id_ = node->get_parameter("behavior_tree_blackboard_id").as_string();

  if (!node->has_parameter("instruction_blackboard_id")) {
    node->declare_parameter("instruction_blackboard_id", std::string("instruction"));
  }
  instruction_blackboard_id_ = node->get_parameter("instruction_blackboard_id").as_string();

  if (!node->has_parameter("plan_blackboard_id")) {
    node->declare_parameter("plan_blackboard_id", std::string("execution_plan"));
  }
  plan_blackboard_id_ = node->get_parameter("plan_blackboard_id").as_string();

  if (!node->has_parameter("domain_file_blackboard_id")) {
    node->declare_parameter("domain_file_blackboard_id", std::string("domain_file"));
  }
  domain_file_blackboard_id_ = node->get_parameter("domain_file_blackboard_id").as_string();

  if (!node->has_parameter("problem_file_blackboard_id")) {
    node->declare_parameter("problem_file_blackboard_id", std::string("problem_file"));
  }
  problem_file_blackboard_id_ = node->get_parameter("problem_file_blackboard_id").as_string();

  self_client_ = rclcpp_action::create_client<ActionT>(node, getName());

  instruction_sub_ = node->create_subscription<std_msgs::msg::String>(
    "instruction",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&TaskPlanner::onInstructionReceived, this, std::placeholders::_1));

  start_bt_sub_ = node->create_subscription<athena_msgs::msg::PlanningProblem>(
    "start_bt",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&TaskPlanner::onStartBtReceived, this, std::placeholders::_1));

  RCLCPP_INFO(logger_, "TaskPlanner configured, listening on 'instruction' and 'start_bt' topics");

  return true;
}

std::string
TaskPlanner::getBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  auto node = parent_node.lock();
  if (!node) {
    return std::string();
  } 

  std::string bt_package, bt_xml_filename;
  node->get_parameter("bt_package_dir", bt_package);
  node->get_parameter("behavior_tree", bt_xml_filename);
  bt_package = ament_index_cpp::get_package_share_directory(bt_package) + "/behavior_trees/";
  bt_xml_filename = bt_package + bt_xml_filename;
  RCLCPP_INFO(logger_, "BT file loaded : %s. ", bt_xml_filename.c_str());
  behavior_tree_ = bt_xml_filename;
  return bt_xml_filename;
}

bool
TaskPlanner::cleanup()
{
  instruction_sub_.reset();
  start_bt_sub_.reset();
  self_client_.reset();
  return true;
}

bool
TaskPlanner::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  auto bt_xml_filename = behavior_tree_;

  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
      logger_, "BT file not found: %s. Generate Actions canceled.",
      bt_xml_filename.c_str());
    return false;
  }

  initializeFromGoal(goal);

  return true;
}

void
TaskPlanner::goalCompleted(
  typename ActionT::Result::SharedPtr result,
  const athena_behavior_tree::BtStatus final_bt_status)
{
  // Populate result based on final BT status
  switch (final_bt_status) {
    case athena_behavior_tree::BtStatus::SUCCEEDED:
      result->error_code = ActionT::Result::NONE;
      result->error_msg = "";
      break;
    case athena_behavior_tree::BtStatus::FAILED:
      result->error_code = 1;
      result->error_msg = "Action generation failed";
      break;
    case athena_behavior_tree::BtStatus::CANCELED:
      result->error_code = 2;
      result->error_msg = "Action generation canceled";
      break;
    default:
      result->error_code = 3;
      result->error_msg = "Unknown termination status";
      break;
  }
}

void
TaskPlanner::onLoop()
{
  auto feedback_msg = std::make_shared<ActionT::Feedback>();
  auto blackboard = bt_action_server_->getBlackboard();

  // Populate current state if available
  try {
    athena_msgs::msg::State current_state;
    blackboard->get<athena_msgs::msg::State>("current_state", current_state);
    feedback_msg->current_state = current_state;
  } catch (...) {
    // Ignore if not on blackboard yet
  }

  // Populate current action if available
  try {
    athena_msgs::msg::Action current_action;
    blackboard->get<athena_msgs::msg::Action>("current_action", current_action);
    feedback_msg->current_action = current_action;
  } catch (...) {
    // Ignore if not on blackboard yet
  }

  // Populate current method if available
  try {
    athena_msgs::msg::Method current_method;
    blackboard->get<athena_msgs::msg::Method>("current_method", current_method);
    feedback_msg->current_method = current_method;
  } catch (...) {
    // Ignore if not on blackboard yet
  }

  bt_action_server_->publishFeedback(feedback_msg);
}

void
TaskPlanner::onPreempt(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(logger_, "Received goal preemption request");

  if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
    (goal->behavior_tree.empty() &&
    bt_action_server_->getCurrentBTFilename() == bt_action_server_->getDefaultBTFilename()))
  {
    initializeFromGoal(bt_action_server_->acceptPendingGoal());
  } else {
    RCLCPP_WARN(
      logger_,
      "Preemption request was rejected since the requested BT XML file is not the same "
      "as the one that the current goal is executing. Preemption with a new BT is invalid "
      "since it would require cancellation of the previous goal instead of true preemption."
      "\nCancel the current goal and send a new action request if you want to use a "
      "different BT XML file. For now, continuing to track the last goal until completion.");
    bt_action_server_->terminatePendingGoal();
  }
}

void
TaskPlanner::initializeFromGoal(ActionT::Goal::ConstSharedPtr goal)
{
  start_time_ = clock_->now();
  auto blackboard = bt_action_server_->getBlackboard();

  // Set behavior tree on blackboard
  blackboard->set<std::string>(behavior_tree_blackboard_id_, goal->behavior_tree);

  // Set the instruction on blackboard
  blackboard->set<std::string>(instruction_blackboard_id_, goal->instruction);

  // Set the domain and problem files on blackboard if they were received on start_bt
  std::optional<athena_msgs::msg::PlanningProblem> problem;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    problem.swap(pending_problem_);
  }
  if (problem) {
    blackboard->set<std::string>(domain_file_blackboard_id_, problem->planning_domain);
    blackboard->set<std::string>(problem_file_blackboard_id_, problem->planning_problem);
    RCLCPP_INFO(
      logger_, "Domain file: %s, problem file: %s",
      problem->planning_domain.c_str(), problem->planning_problem.c_str());
  }

  RCLCPP_INFO(
    logger_, "Begin generating actions for instruction: \"%s\" using BT: %s",
    goal->instruction.c_str(), goal->behavior_tree.c_str());
}

void
TaskPlanner::onInstructionReceived(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(logger_, "Received instruction: \"%s\"", msg->data.c_str());
  last_instruction_ = msg->data;
  sendGoal(msg->data);
}

void
TaskPlanner::onStartBtReceived(const athena_msgs::msg::PlanningProblem::SharedPtr msg)
{
  if (msg->planning_domain.empty() || msg->planning_problem.empty()) {
    RCLCPP_WARN(logger_, "Received start_bt with an empty domain or problem file, BT not started");
    return;
  }

  RCLCPP_INFO(
    logger_, "Received start_bt with domain \"%s\" and problem \"%s\"",
    msg->planning_domain.c_str(), msg->planning_problem.c_str());

  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_problem_ = *msg;
  }
  if (!sendGoal(last_instruction_)) {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_problem_.reset();
  }
}

bool
TaskPlanner::sendGoal(const std::string & instruction)
{
  if (!self_client_->action_server_is_ready()) {
    RCLCPP_WARN(logger_, "Action server '%s' is not ready, goal not sent", getName().c_str());
    return false;
  }

  ActionT::Goal goal;
  goal.instruction = instruction;
  goal.behavior_tree = behavior_tree_;

  self_client_->async_send_goal(goal);
  return true;
}

}  // namespace athena_bt_planner