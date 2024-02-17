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

#ifndef ATHENA_EXE_BT_EXECUTOR__BT_EXECUTOR_HPP_
#define ATHENA_EXE_BT_EXECUTOR__BT_EXECUTOR_HPP_

#include <memory>
#include <string>
#include <vector>

#include "athena_util/lifecycle_node.hpp"
#include "athena_util/odometry_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "athena_execution_bt_executor/executor.hpp"

namespace athena_bt_planner
{

/**
 * @class athena_bt_planner::BtPlanner
 * @brief An action server that uses behavior tree for task planning
 */
class BtExecutor : public athena_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for athena_bt_planner::BtPlanner class
   * @param options Additional options to control creation of the node.
   */
  explicit BtExecutor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief A destructor for athena_bt_planner::BtPlanner class
   */
  ~BtExecutor();

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
  std::unique_ptr<athena_bt_planner::Planner<athena_msgs::action::ComputePlan>> planner_;
  athena_bt_planner::PlannerMuxer plugin_muxer_;

};

}  

#endif 
