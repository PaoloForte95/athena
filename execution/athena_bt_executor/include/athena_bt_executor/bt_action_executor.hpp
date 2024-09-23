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

#ifndef ATHENA_BT_EXECUTOR__BT_ACTION_EXECUTOR_HPP_
#define ATHENA_BT_EXECUTOR__BT_ACTION_EXECUTOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "athena_util/lifecycle_node.hpp"
#include "athena_util/odometry_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "athena_bt_executor/action_executor.hpp"

namespace athena
{
namespace execution{
/**
 * @class athena_bt_planner::BtPlanner
 * @brief An action server that uses behavior tree for task planning
 */
class BtActionExecutor : public athena_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for athena_bt_planner::BtPlanner class
   * @param options Additional options to control creation of the node.
   */
  explicit BtActionExecutor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief A destructor for athena_bt_planner::BtPlanner class
   */
  ~BtActionExecutor();

protected:
  /**
   * @brief Configures member variables
   *
   * Initializes action server for "ComputePlan"; subscription to
   * "task_sub"; and builds behavior tree from xml file.
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  athena_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  athena_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  athena_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Resets member variables
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

  // To handle all the BT related execution
  std::unique_ptr<athena::execution::Executor<athena_msgs::action::ExecuteAction>> executor_;
  athena::execution::ExecutorMuxer plugin_muxer_;

};
}
}  

#endif 
