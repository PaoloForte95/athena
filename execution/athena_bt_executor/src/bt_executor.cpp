/*
Copyright (c) 2024 Paolo Forte

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/



#include <memory>
#include <string>
#include <utility>
#include <set>
#include <limits>
#include <vector>

#include "athena_util/geometry_utils.hpp"
#include "athena_util/robot_utils.hpp"
#include "athena_behavior_tree/bt_conversions.hpp"
#include "athena_bt_executor/bt_executor.hpp"

namespace athena
{
namespace execution{

BtExecutor::BtExecutor(const rclcpp::NodeOptions & options): athena_util::LifecycleNode("bt_executor", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating");

  const std::vector<std::string> plugin_libs = {
    "athena_exe_move_pile_method_node"

  };

  declare_parameter("plugin_lib_names", plugin_libs);
  }

BtExecutor::~BtExecutor()
{
}

athena_util::CallbackReturn
BtExecutor::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Libraries to pull plugins (BT Nodes) from
  auto plugin_lib_names = get_parameter("plugin_lib_names").as_string_array();

  executor_ = std::make_unique<athena::execution::MethodExecutor>();


  if (!executor_->on_configure(
      shared_from_this(), plugin_lib_names, &plugin_muxer_))
  {
    return athena_util::CallbackReturn::FAILURE;
  }


  return athena_util::CallbackReturn::SUCCESS;
}

athena_util::CallbackReturn
BtExecutor::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  if (!executor_->on_activate()) {
    return athena_util::CallbackReturn::FAILURE;
  }

  // create bond connection
  createBond();

  return athena_util::CallbackReturn::SUCCESS;
}

athena_util::CallbackReturn
BtExecutor::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  if (!executor_->on_deactivate()) {
    return athena_util::CallbackReturn::FAILURE;
  }

  // destroy bond connection
  destroyBond();

  return athena_util::CallbackReturn::SUCCESS;
}

athena_util::CallbackReturn
BtExecutor::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");


  if (!executor_->on_cleanup()) {
    return athena_util::CallbackReturn::FAILURE;
  }

  executor_.reset();

  RCLCPP_INFO(get_logger(), "Completed Cleaning up");
  return athena_util::CallbackReturn::SUCCESS;
}

athena_util::CallbackReturn
BtExecutor::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return athena_util::CallbackReturn::SUCCESS;
}

} 
}
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(athena::execution::BtExecutor)
