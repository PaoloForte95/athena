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

#ifndef ATHENA_PLANNER__TASK_PLANNER_SERVER_HPP_
#define ATHENA_PLANNER__TASK_PLANNER_SERVER_HPP_

#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <unordered_map>
#include <mutex>

#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "athena_util/lifecycle_node.hpp"
#include "athena_msgs/action/compute_plan.hpp"
#include "athena_util/robot_utils.hpp"
#include "athena_util/simple_action_server.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "athena_core/planner.hpp"
#include "athena_msgs/msg/plan.hpp"


namespace athena_planner
{
/**
 * @class athena_planner::TaskPlannerServer
 * @brief An action server implements the behavior tree's ComputePathToPose
 * interface and hosts various plugins of different algorithms to compute plans.
 */
class TaskPlannerServer : public athena_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for athena_planner::TaskPlannerServer
   * @param options Additional options to control creation of the node.
   */
  explicit TaskPlannerServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief A destructor for athena_planner::TaskPlannerServer
   */
  ~TaskPlannerServer();

  using PlannerMap = std::unordered_map<std::string, athena_core::Planner::Ptr>;

  /**
   * @brief Get the Execution Plan object
   * 
   * @param domain 
   * @param problem 
   * @param planner_id 
   * @return std::string 
   */
  athena_msgs::msg::Plan getExecutionPlan(
    const std::string & domain,
    const std::string & problem,
    const std::string & planner_id);

protected:
  /**
   * @brief Configure member variables and initializes planner
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  athena_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  athena_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivate member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  athena_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Reset member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  athena_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  athena_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  using ActionPlan = athena_msgs::action::ComputePlan;
  using ActionServerPlan = athena_util::SimpleActionServer<ActionPlan>;

  /**
   * @brief Check if an action server is valid / active
   * @param action_server Action server to test
   * @return SUCCESS or FAILURE
   */
  template<typename T>
  bool isServerInactive(std::unique_ptr<athena_util::SimpleActionServer<T>> & action_server);

  /**
   * @brief Check if an action server has a cancellation request pending
   * @param action_server Action server to test
   * @return SUCCESS or FAILURE
   */
  template<typename T>
  bool isCancelRequested(std::unique_ptr<athena_util::SimpleActionServer<T>> & action_server);


  /**
   * @brief Check if an action server has a preemption request and replaces the goal
   * with the new preemption goal.
   * @param action_server Action server to get updated goal if required
   * @param goal Goal to overwrite
   */
  template<typename T>
  void getPreemptedGoalIfRequested(
    std::unique_ptr<athena_util::SimpleActionServer<T>> & action_server,
    typename std::shared_ptr<const typename T::Goal> goal);



  // Our action server implements the ComputePathToPose action
  std::unique_ptr<ActionServerPlan> action_server_plan_;

  /**
   * @brief 
   * 
   */
  void computeExecutionPlan();

 
  /**
   * @brief Publish the planner for debug purposes
   * @param plann Reference to the execution plan 
   */
  void publishPlan(const athena_msgs::msg::Plan & plan);

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
  pluginlib::ClassLoader<athena_core::Planner> gp_loader_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> planner_ids_;
  std::vector<std::string> planner_types_;
  std::string planner_ids_concat_;

  // Clock
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};


  // Publishers for the path
  rclcpp_lifecycle::LifecyclePublisher<athena_msgs::msg::Plan>::SharedPtr plan_publisher_;


  double planner_frequency_;
};

}  

#endif  