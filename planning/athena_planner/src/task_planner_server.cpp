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

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <utility>

#include "builtin_interfaces/msg/duration.hpp"
#include "athena_util/node_utils.hpp"
#include "athena_util/geometry_utils.hpp"

#include "athena_planner/task_planner_server.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace athena_planner
{

TaskPlannerServer::TaskPlannerServer(const rclcpp::NodeOptions & options)
: athena_util::LifecycleNode("task_planner_server", "", options),
  gp_loader_("athena_core", "athena_core::Planner"),
  default_ids_{"MetricFF", "LPG"},
  default_types_{"athena_planner::LPG", "athena_planner::MetricFF"}
{
  RCLCPP_INFO(get_logger(), "Creating task planner server");

  declare_parameter("planner_frequency", 20.0);
  // Declare this node's parameters
  declare_parameter("planner_plugins", default_ids_);

}

TaskPlannerServer::~TaskPlannerServer()
{
  planners_.clear();

}

athena_util::CallbackReturn
TaskPlannerServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  auto node = shared_from_this();
  RCLCPP_INFO(get_logger(), "Configuring task planner interface");


  get_parameter("planner_plugins", planner_ids_);
  if (planner_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      athena_util::declare_parameter_if_not_declared(
        node, default_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_ids_[i]));
    }
  }
  planner_types_.resize(planner_ids_.size());

  get_parameter("planner_frequency", planner_frequency_);
  RCLCPP_INFO(get_logger(), "Task planner frequency set to %.4fHz", planner_frequency_);


  planner_types_.resize(planner_ids_.size());


  for (size_t i = 0; i != planner_ids_.size(); i++) {
    try {
      planner_types_[i] = athena_util::get_plugin_type_param(
        node, planner_ids_[i]);
      athena_core::Planner::Ptr task_planner =
        gp_loader_.createUniqueInstance(planner_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created task planner plugin %s of type %s",
        planner_ids_[i].c_str(), planner_types_[i].c_str());
      task_planner->configure(node, planner_ids_[i]);
      planners_.insert({planner_ids_[i], task_planner});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create the task planner. Exception: %s",
        ex.what());
      return athena_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    planner_ids_concat_ += planner_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Task Planner Server has %s planners available.", planner_ids_concat_.c_str());

 
  // Initialize pubs & subs
  plan_publisher_ = create_publisher<athena_msgs::msg::Plan>("execution_plan", 1);

  // Create the action servers for path planning to a pose and through poses
  action_server_plan_ = std::make_unique<ActionServerPlan>(
    shared_from_this(),
    "compute_execution_plan",
    std::bind(&TaskPlannerServer::computeExecutionPlan, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);

  return athena_util::CallbackReturn::SUCCESS;
}

athena_util::CallbackReturn
TaskPlannerServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  plan_publisher_->on_activate();
  action_server_plan_->activate();


  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->activate();
  }

  auto node = shared_from_this();


  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&TaskPlannerServer::dynamicParametersCallback, this, _1));

  // create bond connection
  createBond();

  return athena_util::CallbackReturn::SUCCESS;
}

athena_util::CallbackReturn
TaskPlannerServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_plan_->deactivate();
  plan_publisher_->on_deactivate();

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->deactivate();
  }

  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();

  return athena_util::CallbackReturn::SUCCESS;
}

athena_util::CallbackReturn
TaskPlannerServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_plan_.reset();
  plan_publisher_.reset();


  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->cleanup();
  }
  planners_.clear();

  return athena_util::CallbackReturn::SUCCESS;
}

athena_util::CallbackReturn
TaskPlannerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return athena_util::CallbackReturn::SUCCESS;
}

template<typename T>
bool TaskPlannerServer::isServerInactive(
  std::unique_ptr<athena_util::SimpleActionServer<T>> & action_server)
{
  if (action_server == nullptr || !action_server->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return true;
  }

  return false;
}



template<typename T>
bool TaskPlannerServer::isCancelRequested(
  std::unique_ptr<athena_util::SimpleActionServer<T>> & action_server)
{
  if (action_server->is_cancel_requested()) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
    action_server->terminate_all();
    return true;
  }

  return false;
}

template<typename T>
void TaskPlannerServer::getPreemptedGoalIfRequested(
  std::unique_ptr<athena_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}


void
TaskPlannerServer::computeExecutionPlan()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  auto start_time = steady_clock_.now();

  // Initialize the ComputePathToPose goal and result
  auto goal = action_server_plan_->get_current_goal();
  auto result = std::make_shared<ActionPlan::Result>();

  try {
    if (isServerInactive(action_server_plan_) || isCancelRequested(action_server_plan_)) {
      return;
    }

    getPreemptedGoalIfRequested(action_server_plan_, goal);

    RCLCPP_WARN( get_logger(), "Choosen Task Planner: %s ", goal->planner.c_str());

    result->execution_plan = getExecutionPlan(goal->planning_problem.planning_domain, goal->planning_problem.planning_problem,  goal->planner);
    auto message = athena_msgs::msg::Plan();
    message = result->execution_plan;
    // Publish the plan for visualization purposes
    publishPlan(message);

    action_server_plan_->succeeded_current(result);
  } catch (std::exception & ex) {
    RCLCPP_WARN(
      get_logger(), "%s plugin failed to compute execution plan for the problem (%s, %s).",
      goal->planner.c_str(), goal->planning_problem.planning_domain.c_str(), goal->planning_problem.planning_problem.c_str());
    action_server_plan_->terminate_current();
  }
}

athena_msgs::msg::Plan TaskPlannerServer::getExecutionPlan(
    const std::string & domain,
    const std::string & problem,
    const std::string & planner)
{
  athena_msgs::msg::Plan plan;
   RCLCPP_WARN(
      get_logger(), "Attempting to compute an execution plan for the planning problem (%s, %s), using planner %s\"",
      domain.c_str(), problem.c_str(),  planner.c_str());


  if (planners_.find(planner) != planners_.end()) {
    return planners_[planner]->computeExecutionPlan(domain,problem);
  } else {
    if (planners_.size() == 1 && planner.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No planners specified in action call. "
        "Server will use only plugin %s in server."
        " This warning will appear once.", planner_ids_concat_.c_str());
      return planners_[planners_.begin()->first]->computeExecutionPlan(domain, problem);
    } else {
      RCLCPP_ERROR(
        get_logger(), "planner %s is not a valid planner. "
        "Task Planner names are: %s", planner.c_str(),
        planner_ids_concat_.c_str());
    }
  }

  return athena_msgs::msg::Plan();
}

void
TaskPlannerServer::publishPlan(const athena_msgs::msg::Plan & msg)
{
  auto message = std::make_unique<athena_msgs::msg::Plan>(msg);
  if (plan_publisher_->is_activated() && plan_publisher_->get_subscription_count() > 0) {
    plan_publisher_->publish(std::move(message));
  }
}


rcl_interfaces::msg::SetParametersResult
TaskPlannerServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
    }
  }

  result.successful = true;
  return result;
}

}  // namespace athena_planner

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(athena_planner::TaskPlannerServer)
