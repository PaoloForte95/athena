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

#ifndef PLAN2_BT_PLANNER__BT_PLANNER_HPP_
#define PLAN2_BT_PLANNER__BT_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "plan2_util/lifecycle_node.hpp"
#include "plan2_util/odometry_utils.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "plan2_bt_planner/planners/task_planner.hpp"

namespace plan2_bt_planner
{

/**
 * @class plan2_bt_planner::BtPlanner
 * @brief An action server that uses behavior tree for task planning
 */
class BtPlanner : public plan2_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for plan2_bt_planner::BtPlanner class
   * @param options Additional options to control creation of the node.
   */
  explicit BtPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief A destructor for plan2_bt_planner::BtPlanner class
   */
  ~BtPlanner();

protected:
  /**
   * @brief Configures member variables
   *
   * Initializes action server for "ComputePlan"; subscription to
   * "task_sub"; and builds behavior tree from xml file.
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  plan2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  plan2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  plan2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Resets member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  plan2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  plan2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  // To handle all the BT related execution
  std::unique_ptr<plan2_bt_planner::Planner<plan2_msgs::action::ComputePlan>> planner_;
  plan2_bt_planner::PlannerMuxer plugin_muxer_;

  // Odometry smoother object
  std::shared_ptr<plan2_util::OdomSmoother> odom_smoother_;

  // Metrics for feedback
  std::string robot_frame_;
  std::string global_frame_;
  double transform_tolerance_;
  std::string odom_topic_;

  // Spinning transform that can be used by the BT nodes
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

}  // namespace plan2_bt_planner

#endif  // PLAN2_BT_PLANNER__BT_PLANNER_HPP_
