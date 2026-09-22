#ifndef ATHENA_BT_PLANNER__PLANNERS__TASK_PLANNER_HPP_
#define ATHENA_BT_PLANNER__PLANNERS__TASK_PLANNER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <optional>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "athena_bt_planner/behavior_tree_planner.hpp"
#include "std_msgs/msg/string.hpp"
#include "athena_msgs/action/generate_tasks.hpp"
#include "athena_msgs/msg/planning_problem.hpp"
#include "athena_msgs/msg/plan.hpp"
#include "athena_msgs/msg/state.hpp"
#include "athena_msgs/msg/action.hpp"
#include "athena_msgs/msg/method.hpp"
#include "athena_util/geometry_utils.hpp"
#include "athena_util/robot_utils.hpp"

namespace athena_bt_planner
{

/**
 * @class TaskPlanner
 * @brief A planner for task planning that generates actions from instructions
 */
class TaskPlanner: public athena_bt_planner::Planner<athena_msgs::action::GenerateTasks>{
public:
  using ActionT = athena_msgs::action::GenerateTasks;

  /**
   * @brief A constructor for TaskPlanner
   */
  TaskPlanner() = default;

  /**
   * @brief A configure state transition to configure planner's state
   * @param node Weakptr to the lifecycle node
   */
  bool configure(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

  /**
   * @brief A cleanup state transition to remove memory allocated
   */
  bool cleanup() override;

  /**
   * @brief A subscription and callback to handle the topic-based instruction published
   * @param msg Instruction received via a topic
   */
  void onInstructionReceived(const std_msgs::msg::String::SharedPtr msg);

  /**
   * @brief A subscription and callback that starts the behavior tree when a planning
   * problem is received on the start_bt topic. The domain and problem files are put on
   * the blackboard when the goal starts. The last received instruction is used.
   * @param msg Planning domain and problem files received via a topic
   */
  void onStartBtReceived(const athena_msgs::msg::PlanningProblem::SharedPtr msg);

  /**
   * @brief Get action name for this planner
   * @return string Name of action server
   */
  std::string getName() override {return std::string("task_planner");}

  /**
   * @brief Get planner's BT
   * @param node WeakPtr to the lifecycle node
   * @return string Filepath to XML bt
   */
  std::string getBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) override;

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
   * @brief Generate actions from the instruction and BT on the blackboard
   * @param goal Action template's goal message to process
   */
  void initializeFromGoal(ActionT::Goal::ConstSharedPtr goal);

  /**
   * @brief Send a goal to this planner's own action server, which starts the BT
   * @param instruction Instruction to put in the goal
   * @return bool if the goal was sent
   */
  bool sendGoal(const std::string & instruction);

  rclcpp::Time start_time_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr instruction_sub_;
  rclcpp::Subscription<athena_msgs::msg::PlanningProblem>::SharedPtr start_bt_sub_;
  rclcpp_action::Client<ActionT>::SharedPtr self_client_;

  std::string behavior_tree_blackboard_id_;
  std::string instruction_blackboard_id_;
  std::string plan_blackboard_id_;
  std::string domain_file_blackboard_id_;
  std::string problem_file_blackboard_id_;
  std::string behavior_tree_;
  std::string last_instruction_;

  std::mutex pending_mutex_;
  std::optional<athena_msgs::msg::PlanningProblem> pending_problem_;
};

} 

#endif