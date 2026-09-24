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

#include "athena_util/geometry_utils.hpp"
#include "athena_util/robot_utils.hpp"
#include "athena_behavior_tree/bt_conversions.hpp"
#include "athena_bt_planner/bt_planner.hpp"

namespace athena_bt_planner
{

BtPlanner::BtPlanner(const rclcpp::NodeOptions & options)
: athena_util::LifecycleNode("bt_planner", "", options)
{
  const std::vector<std::string> plugin_libs = {
    "athena_compute_plan_action_bt_node"

  };

  declare_parameter("plugin_lib_names", plugin_libs);
  declare_parameter("behavior_tree", std::string(""));
  declare_parameter("bt_package_dir", std::string("athena_behavior_tree"));

}

BtPlanner::~BtPlanner() = default;

athena_util::CallbackReturn
BtPlanner::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");



  // Libraries to pull plugins (BT Nodes) from
  auto plugin_lib_names = get_parameter("plugin_lib_names").as_string_array();

  planner_ = std::make_unique<athena_bt_planner::TaskPlanner>();

  if (!planner_->on_configure(shared_from_this(), plugin_lib_names, &plugin_muxer_))
  {
    return athena_util::CallbackReturn::FAILURE;
  }


  return athena_util::CallbackReturn::SUCCESS;
}

athena_util::CallbackReturn
BtPlanner::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  if (!planner_->on_activate()) {
    return athena_util::CallbackReturn::FAILURE;
  }

  // create bond connection
  createBond();

  return athena_util::CallbackReturn::SUCCESS;
}

athena_util::CallbackReturn
BtPlanner::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  if (!planner_->on_deactivate()) {
    return athena_util::CallbackReturn::FAILURE;
  }

  // destroy bond connection
  destroyBond();

  return athena_util::CallbackReturn::SUCCESS;
}

athena_util::CallbackReturn
BtPlanner::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  if (!planner_->on_cleanup()) {
    return athena_util::CallbackReturn::FAILURE;
  }

  planner_.reset();

  RCLCPP_INFO(get_logger(), "Completed Cleaning up");
  return athena_util::CallbackReturn::SUCCESS;
}

athena_util::CallbackReturn
BtPlanner::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return athena_util::CallbackReturn::SUCCESS;
}

} 

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(athena_bt_planner::BtPlanner)
