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

#ifndef PLAN2_BT_PLANNER__PLANNERS__TASK_PLANNER_HPP_
#define PLAN2_BT_PLANNER__PLANNERS__TASK_PLANNER_HPP_

#include <string>
#include <vector>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "plan2_bt_planner/planner.hpp"
#include "plan2_msgs/msg/planning_problem.hpp"
#include "plan2_msgs/action/compute_plan.hpp"
#include "plan2_msgs/msg/plan.hpp"
#include "plan2_util/geometry_utils.hpp"
#include "plan2_util/robot_utils.hpp"

namespace plan2_bt_planner
{

/**
 * @class TaskPlanner
 * @brief A planner for task planning 
 */
class TaskPlanner
  : public plan2_bt_planner::Planner<plan2_msgs::action::ComputePlan>
{
public:
  using ActionT = plan2_msgs::action::ComputePlan;

  /**
   * @brief A constructor for TaskPlanner
   */
  TaskPlanner()
  : Planner() {}

  /**
   * @brief A configure state transition to configure planner's state
   * @param node Weakptr to the lifecycle node
   * @param odom_smoother Object to get current smoothed robot's speed
   */
  bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr node, std::shared_ptr<plan2_util::OdomSmoother> odom_smoother) override;

  /**
   * @brief A cleanup state transition to remove memory allocated
   */
  bool cleanup() override;

    /**
   * @brief A subscription and callback to handle the topic-based planning problem published
   * @param pose Pose received via atopic
   */
  void onPlanningProblemReceived(const plan2_msgs::msg::PlanningProblem::SharedPtr pose);

  /**
   * @brief Get action name for this planner
   * @return string Name of action server
   */
  std::string getName() {return std::string("task_planner");}

  /**
   * @brief Get planner's default BT
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
    const plan2_behavior_tree::BtStatus final_bt_status) override;

  /**
   * @brief planning problem initialization on the blackboard
   * @param goal Action template's goal message to process
   */
  void initializePlanningProblem(ActionT::Goal::ConstSharedPtr goal);

  rclcpp::Time start_time_;

  rclcpp::Subscription<plan2_msgs::msg::PlanningProblem>::SharedPtr planning_problem_sub_;
  rclcpp_action::Client<ActionT>::SharedPtr self_client_;

  std::string domain_file_blackboard_id_;
  std::string problem_file_blackboard_id_;
  std::string plan_blackboard_id_;

  // Odometry smoother object
  std::shared_ptr<plan2_util::OdomSmoother> odom_smoother_;
};

}  // namespace plan2_bt_planner

#endif  // PLAN2_BT_PLANNER__PLANNERS__TASK_PLANNER_HPP_
