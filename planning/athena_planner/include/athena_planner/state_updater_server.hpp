// Copyright (c) 2024 Paolo Forte
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

#ifndef ATHENA_PLANNER__STATE_UPDATER_SERVER_HPP_
#define ATHENA_PLANNER__STATE_UPDATER_SERVER_HPP_

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
#include "athena_msgs/action/update_state.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "athena_core/state_updater.hpp"
#include "athena_msgs/msg/state.hpp"
#include "athena_util/simple_action_server.hpp"
typedef std::vector<athena_msgs::msg::Action> Actions;

namespace athena_planner
{
/**
 * @class athena_planner::StateUpdaterServer
 * @brief An action server implements the behavior tree's ComputePathToPose
 * interface and hosts various plugins of different algorithms to compute plans.
 */
class StateUpdaterServer : public athena_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for athena_planner::StateUpdaterServer
   * @param options Additional options to control creation of the node.
   */
  explicit StateUpdaterServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief A destructor for athena_planner::StateUpdaterServer
   */
  ~StateUpdaterServer();

  using StateUpdaterMap = std::unordered_map<std::string, athena_core::StateUpdater::Ptr>;

  
  
  void updateState();


  athena_msgs::msg::State getUpdatedState(
    const athena_msgs::msg::State & previous_state,
    const Actions & actions,
    const std::string & state_updater);

protected:
  /**
   * @brief Configure member variables and initializes state updater
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

  using ActionUpdate = athena_msgs::action::UpdateState;
  using ActionServerUpdate = athena_util::SimpleActionServer<ActionUpdate>;

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



  // Our action server implements the UpdateState action
  std::unique_ptr<ActionServerUpdate> action_server_update_;


  /**
   * @brief Publish the update state of the planning problem
   * 
   * @param state the current state to publish
   */
  void publishState(const athena_msgs::msg::State & state);

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult
  dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

  // Dynamic parameters handler
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;
  std::mutex dynamic_params_lock_;

  // Updaters
  StateUpdaterMap state_updaters_;
  pluginlib::ClassLoader<athena_core::StateUpdater> gp_loader_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> state_updater_ids_;
  std::vector<std::string> state_updater_types_;
  std::string state_updater_ids_concat_;

  // Clock
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};


  // Publishers for the path
  rclcpp_lifecycle::LifecyclePublisher<athena_msgs::msg::State>::SharedPtr state_publisher_;


  double state_updater_frequency_;
};

}  

#endif  