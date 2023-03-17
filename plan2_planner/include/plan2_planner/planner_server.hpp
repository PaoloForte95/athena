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

#ifndef PLAN2_PLANNER__PLANNER_SERVER_HPP_
#define PLAN2_PLANNER__PLANNER_SERVER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <mutex>


#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "plan2_core/planner.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/simple_action_server.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "plan2_msgs/action/compute_plan.hpp"
#include "plan2_msgs/msg/plan.hpp"


namespace plan2_planner
{

/**
 * @class plan2_planner::EclPlannerServer
 * @brief An action server implements the behavior tree's ComputePathToPose
 * interface and hosts various plugins of different algorithms to compute plans.
 */
class PlannerServer : public nav2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for orunav2_planner::EclPlannerServer
   * @param options Additional options to control creation of the node.
   */
  explicit PlannerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief A destructor for orunav2_planner::EclPlannerServer
   */
  ~PlannerServer();

  using PlannerMap = std::unordered_map<std::string, plan2_core::Planner::Ptr>;

  /**
   * @brief Method to compute the execution Plan
   * @param start starting pose
   * @param goal goal request
   * @return Path
   */
  plan2_msgs::msg::Plan getExecutionPlan(
    const std::string & domain,
    const std::string & problem,
    const std::string & planner);

protected:
  /**
   * @brief Configure member variables and initializes planner
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Reset member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  using ActionComputePlan = plan2_msgs::action::ComputePlan;
  using ActionServerComputePlan = nav2_util::SimpleActionServer<ActionComputePlan>;

  /**
   * @brief Check if an action server is valid / active
   * @param action_server Action server to test
   * @return SUCCESS or FAILURE
   */
  template<typename T>
  bool isServerInactive(std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server);

  /**
   * @brief Check if an action server has a cancellation request pending
   * @param action_server Action server to test
   * @return SUCCESS or FAILURE
   */
  template<typename T>
  bool isCancelRequested(std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server);

  /**
   * @brief Check if an action server has a preemption request and replaces the goal
   * with the new preemption goal.
   * @param action_server Action server to get updated goal if required
   * @param goal Goal to overwrite
   */
  template<typename T>
  void getPreemptedGoalIfRequested(
    std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
    typename std::shared_ptr<const typename T::Goal> goal);


  // Our action server implements the ComputePlan action
  std::unique_ptr<ActionServerComputePlan> action_server_plan_;


  /**
   * @brief The action server callback which calls planner to get the execution plan
   * ComputePathToPose
   */
  void computeExecutionPlan();

  /**
   * @brief Publish a plan for debug purpose
   * @param path Reference to the execution plan
   */
  void publishPlan(const plan2_msgs::msg::Plan & plan);

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;

  // Planner
  PlannerMap planners_;
  pluginlib::ClassLoader<plan2_core::Planner> gp_loader_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> planner_ids_;
  std::vector<std::string> planner_types_;
  double max_planner_duration_;
  std::string planner_ids_concat_;

  // Clock
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};


  // Publishers for the path
  rclcpp_lifecycle::LifecyclePublisher<plan2_msgs::msg::Plan>::SharedPtr plan_publisher_;

};


}  // namespace plan2_planner

#endif  // PLAN2_PLANNER__PLANNER_SERVER_HPP_
