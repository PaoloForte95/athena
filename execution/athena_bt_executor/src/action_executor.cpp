/*
Copyright (c) 2024 Paolo Forte

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

#include <vector>
#include <string>
#include <set>
#include <memory>
#include <limits>
#include "athena_bt_executor/action_executor.hpp"

namespace athena
{
  namespace execution
  {
    bool
    ActionExecutor::configure(
        rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
    {
      start_time_ = rclcpp::Time(0);
      auto node = parent_node.lock();

      if (!node->has_parameter("actions_blackboard_id"))
      {
        node->declare_parameter("actions_blackboard_id", std::string("actions"));
      }

      actions_blackboard_id_ = node->get_parameter("actions_blackboard_id").as_string();


      self_client_ = rclcpp_action::create_client<ActionT>(node, getName());

      action_server_ = rclcpp_action::create_server<ActionT>(
          node,
          "method",
          std::bind(&ActionExecutor::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
          std::bind(&ActionExecutor::handle_cancel, this, std::placeholders::_1),
          std::bind(&ActionExecutor::handle_accepted, this, std::placeholders::_1));

      return true;
    }

    rclcpp_action::GoalResponse ActionExecutor::handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ActionT::Goal> goal)
    {
      RCLCPP_INFO(logger_, "Received goal request for action %s with id %d",goal->action.name.c_str(), goal->action.action_id);
      (void)uuid;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse ActionExecutor::handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
    {
      RCLCPP_INFO(logger_, "Received request to cancel method execution");
      (void)goal_handle;
      return rclcpp_action::CancelResponse::ACCEPT;
    }

    void ActionExecutor::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
    {
      // this needs to return quickly to avoid blocking the executor, so spin up a new thread
      std::thread{std::bind(&ActionExecutor::Execute, this, std::placeholders::_1), goal_handle}.detach();
    }

    std::string
    ActionExecutor::getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
    {
      auto node = parent_node.lock();

      node->declare_parameter<std::string>("bt_package_dir", "athena_exe_behavior_tree");
      node->declare_parameter<std::string>("default_plan_bt_xml", "execution.xml");
      std::string bt_package, bt_xml_filename;
      node->get_parameter("bt_package_dir", bt_package);
      node->get_parameter("default_plan_bt_xml", bt_xml_filename);
      bt_package = ament_index_cpp::get_package_share_directory(bt_package) + "/behavior_trees/";
      bt_xml_filename = bt_package + bt_xml_filename;
      RCLCPP_INFO(logger_, "BT file loaded : %s. ", bt_xml_filename.c_str());
      return bt_xml_filename;
    }

    bool
    ActionExecutor::cleanup()
    {

      self_client_.reset();
      return true;
    }

    bool
    ActionExecutor::goalReceived(ActionT::Goal::ConstSharedPtr goal)
    {
      auto bt_xml_filename = goal->behavior_tree;

      if (!bt_action_server_->loadBehaviorTree(bt_xml_filename))
      {
        RCLCPP_ERROR(
            logger_, "BT file not found: %s. Task Planning canceled.",
            bt_xml_filename.c_str());
        return false;
      }

      initializeExecution(goal);

      return true;
    }

    void
    ActionExecutor::goalCompleted(
        typename ActionT::Result::SharedPtr /*result*/,
        const athena_behavior_tree::BtStatus /*final_bt_status*/)
    {
    }

    void
    ActionExecutor::onLoop()
    {
      // action server feedback (pose, duration of task, number of recoveries, and distance remaining to goal)
      auto feedback_msg = std::make_shared<ActionT::Feedback>();

      auto blackboard = bt_action_server_->getBlackboard();

      int recovery_count = 0;
      blackboard->get<int>("number_recoveries", recovery_count);

      //feedback_msg->current_action_id = 1;

      bt_action_server_->publishFeedback(feedback_msg);
    }

    void
    ActionExecutor::onPreempt(ActionT::Goal::ConstSharedPtr goal)
    {
      RCLCPP_INFO(logger_, "Received goal preemption request");

      if (goal->behavior_tree == bt_action_server_->getCurrentBTFilename() ||
          (goal->behavior_tree.empty() &&
           bt_action_server_->getCurrentBTFilename() == bt_action_server_->getDefaultBTFilename()))
      {
        // if pending goal requests the same BT as the current goal, accept the pending goal
        // if pending goal has an empty behavior_tree field, it requests the default BT file
        // accept the pending goal if the current goal is running the default BT file
        initializeExecution(bt_action_server_->acceptPendingGoal());
      }
      else
      {
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
    ActionExecutor::initializeExecution(ActionT::Goal::ConstSharedPtr goal)
    {
      RCLCPP_INFO(
          logger_, "Begin executing action %s with ID %d",
          goal->action.name.c_str(),goal->action.action_id);

      // Reset state for new action feedback
      start_time_ = clock_->now();
      auto blackboard = bt_action_server_->getBlackboard();
      blackboard->set<int>("number_recoveries", 0); // NOLINT

      // Update the action to execute on the blackboard
      std::vector<athena_msgs::msg::Action> actions;
      actions.push_back( goal->action);
      blackboard->set<std::vector<athena_msgs::msg::Action>>(actions_blackboard_id_, actions);
      RCLCPP_INFO(
          logger_, "Begin executing action %s with ID %d",
          goal->action.name.c_str(),goal->action.action_id);
    }

    void ActionExecutor::Execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)
    {
      ActionT::Goal goal;
      goal.behavior_tree = goal_handle->get_goal()->behavior_tree;
      goal.action = goal_handle->get_goal()->action;
      self_client_->async_send_goal(goal);
    }

  }
}
