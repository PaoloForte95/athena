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



#include <memory>
#include <string>
#include <utility>
#include <set>
#include <limits>
#include <vector>

#include "plan2_util/geometry_utils.hpp"
#include "plan2_util/robot_utils.hpp"
#include "plan2_behavior_tree/bt_conversions.hpp"
#include "plan2_bt_planner/bt_planner.hpp"

namespace plan2_bt_planner
{

BtPlanner::BtPlanner(const rclcpp::NodeOptions & options)
: plan2_util::LifecycleNode("bt_planner", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating");

  const std::vector<std::string> plugin_libs = {
    "plan2_compute_plan_action_bt_node",
    "plan2_is_action_condition_bt_node"

  };

  declare_parameter("plugin_lib_names", plugin_libs);
  declare_parameter("transform_tolerance", rclcpp::ParameterValue(0.1));
  declare_parameter("global_frame", std::string("map"));
  declare_parameter("robot_base_frame", std::string("base_link"));
  declare_parameter("odom_topic", std::string("odom"));
}

BtPlanner::~BtPlanner()
{
}

plan2_util::CallbackReturn
BtPlanner::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  tf_->setUsingDedicatedThread(true);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);

  global_frame_ = get_parameter("global_frame").as_string();
  robot_frame_ = get_parameter("robot_base_frame").as_string();
  transform_tolerance_ = get_parameter("transform_tolerance").as_double();
  odom_topic_ = get_parameter("odom_topic").as_string();

  // Libraries to pull plugins (BT Nodes) from
  auto plugin_lib_names = get_parameter("plugin_lib_names").as_string_array();

  planner_ = std::make_unique<plan2_bt_planner::TaskPlanner>();


  // Odometry smoother object for getting current speed
  odom_smoother_ = std::make_shared<plan2_util::OdomSmoother>(shared_from_this(), 0.3, odom_topic_);

  if (!planner_->on_configure(
      shared_from_this(), plugin_lib_names, &plugin_muxer_, odom_smoother_))
  {
    return plan2_util::CallbackReturn::FAILURE;
  }


  return plan2_util::CallbackReturn::SUCCESS;
}

plan2_util::CallbackReturn
BtPlanner::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  if (!planner_->on_activate()) {
    return plan2_util::CallbackReturn::FAILURE;
  }

  // create bond connection
  createBond();

  return plan2_util::CallbackReturn::SUCCESS;
}

plan2_util::CallbackReturn
BtPlanner::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  if (!planner_->on_deactivate()) {
    return plan2_util::CallbackReturn::FAILURE;
  }

  // destroy bond connection
  destroyBond();

  return plan2_util::CallbackReturn::SUCCESS;
}

plan2_util::CallbackReturn
BtPlanner::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // Reset the listener before the buffer
  tf_listener_.reset();
  tf_.reset();

  if (!planner_->on_cleanup()) {
    return plan2_util::CallbackReturn::FAILURE;
  }

  planner_.reset();

  RCLCPP_INFO(get_logger(), "Completed Cleaning up");
  return plan2_util::CallbackReturn::SUCCESS;
}

plan2_util::CallbackReturn
BtPlanner::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return plan2_util::CallbackReturn::SUCCESS;
}

}  // namespace plan2_bt_planner

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(plan2_bt_planner::BtPlanner)
