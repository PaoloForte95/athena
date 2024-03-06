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

#ifndef ATHENA_BT_EXECUTOR__METHOD_EXECUTOR_HPP_
#define ATHENA_BT_EXECUTOR__METHOD_EXECUTOR_HPP_

#include <string>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "athena_bt_executor/executor.hpp"
#include "athena_msgs/action/execute_method.hpp"
#include "athena_util/geometry_utils.hpp"
#include "athena_util/robot_utils.hpp"

namespace athena
{
namespace execution{
/**
 * @class MethodExecutor
 * @brief An executor for hddl methods 
 */
class MethodExecutor: public Executor<athena_msgs::action::ExecuteMethod>
{
public:
  using ActionT = athena_msgs::action::ExecuteMethod;

  /**
   * @brief A constructor for MethodExecutor
   */
  MethodExecutor()
  : Executor() {}

  /**
   * @brief A configure state transition to configure executor's state
   * @param node Weakptr to the lifecycle node
   */
  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

  /**
   * @brief A cleanup state transition to remove memory allocated
   */
  bool cleanup() override;

    /**
   * @brief A subscription and callback to handle the topic-based planning problem published
   * @param pose Pose received via atopic
   */
  void onExecutionReceived(const athena_msgs::msg::Execution::SharedPtr method);

  /**
   * @brief Get action name for this executor
   * @return string Name of action server
   */
  std::string getName() {return std::string("method_executor");}

  /**
   * @brief Get executor's default BT
   * @param node WeakPtr to the lifecycle node
   * @return string Filepath to default XML
   */
  std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

protected:
  /**
   * @brief A callback to be called when a new goal is received by the BT action server
   * Can be used to check if goal is valid and put values on
   * the blackboard which depend on the received goal
   * @param goal Action template's goal message
   * @return bool if goal was received successfully to be processed
   */
  bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;

  /**
   * @brief A callback that defines execution that happens on one iteration through the BT
   * Can be used to publish action feedback
   */
  void onLoop() override;

  /**
   * @brief A callback that is called when a preempt is requested
   */
  void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;

  /**
   * @brief A callback that is called when a the action is completed, can fill in
   * action result message or indicate that this action is done.
   * @param result Action template result message to populate
   * @param final_bt_status Resulting status of the behavior tree execution that may be
   * referenced while populating the result.
   */
  void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const athena_behavior_tree::BtStatus final_bt_status) override;

  /**
   * @brief planning problem initialization on the blackboard
   * @param goal Action template's goal message to process
   */
  void initializeExecution(ActionT::Goal::ConstSharedPtr goal);


  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const ActionT::Goal> goal);

  void handle_accepted(const std::shared_ptr< rclcpp_action::ServerGoalHandle<ActionT>> goal_handle);

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle);

  void Execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle);

  rclcpp::Time start_time_;

  rclcpp::Subscription<athena_msgs::msg::Execution>::SharedPtr execution_sub_;
  rclcpp_action::Client<ActionT>::SharedPtr self_client_;
  rclcpp_action::Server<ActionT>::SharedPtr action_server_;
  std::string method_blackboard_id_;
  std::string actions_blackboard_id_;

};

}

} 

#endif
