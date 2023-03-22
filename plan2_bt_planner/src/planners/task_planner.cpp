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

#include <vector>
#include <string>
#include <set>
#include <memory>
#include <limits>
#include "plan2_bt_planner/planners/task_planner.hpp"

namespace plan2_bt_planner
{

bool
TaskPlanner::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<plan2_util::OdomSmoother> odom_smoother)
{
  start_time_ = rclcpp::Time(0);
  auto node = parent_node.lock();

  if (!node->has_parameter("domain_file_blackboard_id")) {
    node->declare_parameter("domain_file_blackboard_id", std::string("domain_file"));
  }

  domain_file_blackboard_id_ = node->get_parameter("domain_file_blackboard_id").as_string();

  if (!node->has_parameter("problem_file_blackboard_id")) {
    node->declare_parameter("problem_file_blackboard_id", std::string("problem_file"));
  }

  problem_file_blackboard_id_ = node->get_parameter("problem_file_blackboard_id").as_string();

  if (!node->has_parameter("plan_blackboard_id")) {
    node->declare_parameter("plan_blackboard_id", std::string("execution_plan"));
  }

  plan_blackboard_id_ = node->get_parameter("plan_blackboard_id").as_string();

  // Odometry smoother object for getting current speed
  odom_smoother_ = odom_smoother;

  self_client_ = rclcpp_action::create_client<ActionT>(node, getName());

  planning_problem_sub_ = node->create_subscription<plan2_msgs::msg::PlanningProblem>(
    "planning_problem",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&TaskPlanner::onGoalPoseReceived, this, std::placeholders::_1));
  return true;
}

std::string
TaskPlanner::getDefaultBTFilepath(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  std::string default_bt_xml_filename;
  auto node = parent_node.lock();

  if (!node->has_parameter("default_plan_bt_xml")) {
    std::string pkg_share_dir =
      ament_index_cpp::get_package_share_directory("plan2_behavior_tree");
    node->declare_parameter<std::string>(
      "default_plan_bt_xml",
      pkg_share_dir +
      "/behavior_trees/planning_w_execution.xml");
  }

  node->get_parameter("default_plan_bt_xml", default_bt_xml_filename);

  return default_bt_xml_filename;
}

bool
TaskPlanner::cleanup()
{
  planning_problem_sub_.reset();
  self_client_.reset();
  return true;
}

bool
TaskPlanner::goalReceived(ActionT::Goal::ConstSharedPtr goal)
{
  auto bt_xml_filename = goal->behavior_tree;

  if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(
      logger_, "BT file not found: %s. Task Planning canceled.",
      bt_xml_filename.c_str());
    return false;
  }

  initializePlanningProblem(goal);

  return true;
}

void
TaskPlanner::goalCompleted(
  typename ActionT::Result::SharedPtr /*result*/,
  const plan2_behavior_tree::BtStatus /*final_bt_status*/)
{
}

void
TaskPlanner::onLoop()
{
  // action server feedback (pose, duration of task, number of recoveries, and distance remaining to goal)
  auto feedback_msg = std::make_shared<ActionT::Feedback>();

  auto blackboard = bt_action_server_->getBlackboard();

  try {
    // Get current path points
    plan2_msgs::msg::Plan current_plan;
    blackboard->get<plan2_msgs::msg::Plan>(plan_blackboard_id_, current_plan);

  } catch (...) {
    // Ignore
  }

  int recovery_count = 0;
  blackboard->get<int>("number_recoveries", recovery_count);

  feedback_msg->planning_time = clock_->now() - start_time_;

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
    // if pending goal requests the same BT as the current goal, accept the pending goal
    // if pending goal has an empty behavior_tree field, it requests the default BT file
    // accept the pending goal if the current goal is running the default BT file
    initializePlanningProblem(bt_action_server_->acceptPendingGoal());
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
TaskPlanner::initializePlanningProblem(ActionT::Goal::ConstSharedPtr goal)
{
  RCLCPP_INFO(
    logger_, "Begin task planning for domain file (%s) and problem file (%s)",
    goal->planning_problem.planning_domain.c_str(), goal->planning_problem.planning_problem.c_str());

  // Reset state for new action feedback
  start_time_ = clock_->now();
  auto blackboard = bt_action_server_->getBlackboard();
  blackboard->set<int>("number_recoveries", 0);  // NOLINT

  // Update the goal pose on the blackboard
  blackboard->set<std::string>(domain_file_blackboard_id_, goal->planning_problem.planning_domain);
  blackboard->set<std::string>(problem_file_blackboard_id_, goal->planning_problem.planning_problem);

}

void
TaskPlanner::onGoalPoseReceived(const plan2_msgs::msg::PlanningProblem::SharedPtr problem)
{
  ActionT::Goal goal;
  goal.planning_problem = *problem;
  self_client_->async_send_goal(goal);
}

}  // namespace plan2_bt_planner
