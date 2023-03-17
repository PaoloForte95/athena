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
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "plan2_planner/planner_server.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace plan2_planner
{

PlannerServer::PlannerServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("planner_server", "", options),
  gp_loader_("plan2_core", "plan2_core::Planner"),
  default_ids_{"SymbolicPlanner"},
  default_types_{"plan2_planner/MetricFFPlanner", "plan2_planner/LPGPlanner"}
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare this node's parameters
  declare_parameter("planner_plugins", default_ids_);
  declare_parameter("expected_planner_frequency", 1.0);

  get_parameter("planner_plugins", planner_ids_);
  if (planner_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      declare_parameter(default_ids_[i] + ".plugin", default_types_[i]);
    }
  }


}

PlannerServer::~PlannerServer()
{
  planners_.clear();
}

nav2_util::CallbackReturn
PlannerServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");


  planner_types_.resize(planner_ids_.size());

  auto node = shared_from_this();

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    try {
      planner_types_[i] = nav2_util::get_plugin_type_param(
        node, planner_ids_[i]);
      plan2_core::Planner::Ptr planner =
        gp_loader_.createUniqueInstance(planner_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created global planner plugin %s of type %s",
        planner_ids_[i].c_str(), planner_types_[i].c_str());
      planner->configure(node, planner_ids_[i]);
      planners_.insert({planner_ids_[i], planner});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create global planner. Exception: %s",
        ex.what());
      return nav2_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != planner_ids_.size(); i++) {
    planner_ids_concat_ += planner_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Planner Server has %s planners available.", planner_ids_concat_.c_str());

  double expected_planner_frequency;
  get_parameter("expected_planner_frequency", expected_planner_frequency);
  if (expected_planner_frequency > 0) {
    max_planner_duration_ = 1 / expected_planner_frequency;
  } else {
    RCLCPP_WARN(
      get_logger(),
      "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
      " than 0.0 to turn on duration overrrun warning messages", expected_planner_frequency);
    max_planner_duration_ = 0.0;
  }

  // Initialize pubs & subs
  plan_publisher_ = create_publisher<plan2_msgs::msg::Plan>("execution_plan", 1);

  // Create the action servers for path planning to a pose and through poses
  action_server_plan_ = std::make_unique<ActionServerComputePlan>(
    shared_from_this(),
    "compute_path_to_pose",
    std::bind(&PlannerServer::computeExecutionPlan, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);


  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
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
    std::bind(&PlannerServer::dynamicParametersCallback, this, _1));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
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

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_plan_.reset();
  plan_publisher_.reset();

  PlannerMap::iterator it;
  for (it = planners_.begin(); it != planners_.end(); ++it) {
    it->second->cleanup();
  }
  planners_.clear();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
PlannerServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

template<typename T>
bool PlannerServer::isServerInactive(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server == nullptr || !action_server->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return true;
  }

  return false;
}

template<typename T>
bool PlannerServer::isCancelRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server)
{
  if (action_server->is_cancel_requested()) {
    RCLCPP_INFO(get_logger(), "Goal was canceled. Canceling planning action.");
    action_server->terminate_all();
    return true;
  }

  return false;
}

template<typename T>
void PlannerServer::getPreemptedGoalIfRequested(
  std::unique_ptr<nav2_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}




void
PlannerServer::computeExecutionPlan()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  auto start_time = steady_clock_.now();

  // Initialize the ComputePathToPose goal and result
  auto goal = action_server_plan_->get_current_goal();
  auto result = std::make_shared<ActionComputePlan::Result>();

  try {
    if (isServerInactive(action_server_plan_) || isCancelRequested(action_server_plan_)) {
      return;
    }


    getPreemptedGoalIfRequested(action_server_plan_, goal);

    // Get the planning domain
    std::string domain = goal->planning_domain;
    if (domain.empty()) {
      return;
    }

    //Get the planning problem
     std::string problem = goal->planning_problem;
    if (problem.empty()) {
      return;
    }

    RCLCPP_INFO(get_logger(), " Computing the execution plan with planner %s", goal->planner.c_str());
    result->execution_plan = getExecutionPlan(domain, problem, goal->planner);



    // Publish the plan for visualization purposes
    publishPlan(result->execution_plan);

    auto cycle_duration = steady_clock_.now() - start_time;


    if (max_planner_duration_ && cycle_duration.seconds() > max_planner_duration_) {
      RCLCPP_WARN(
        get_logger(),
        "Planner loop missed its desired rate of %.4f Hz. Current loop rate is %.4f Hz",
        1 / max_planner_duration_, 1 / cycle_duration.seconds());
    }

    action_server_plan_->succeeded_current(result);
  } catch (std::exception & ex) {
    RCLCPP_WARN(
      get_logger(), "%s plugin failed to compute execution plan using planner %s, domain %s, and problem %s (%.2f, %.2f): \"%s\"",
      goal->planner.c_str(), goal->planning_domain.c_str(),
      goal->planning_problem.c_str(), ex.what());
    action_server_plan_->terminate_current();
  }
}

plan2_msgs::msg::Plan
PlannerServer::getExecutionPlan(
  const std::string & domain,
  const std::string & problem,
  const std::string & planner)
{
  RCLCPP_DEBUG(
    get_logger(), "Attempting to a compute an execution plan");

  if (planners_.find(planner) != planners_.end()) {

    return planners_[planner]->computeExecutionPlan(domain, problem);
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
        "Planner names are: %s", planner.c_str(),
        planner_ids_concat_.c_str());
    }
  }

  return plan2_msgs::msg::Plan();
}

void
PlannerServer::publishPlan(const plan2_msgs::msg::Plan & path)
{
  auto msg = std::make_unique<plan2_msgs::msg::Plan>(path);
  if (plan_publisher_->is_activated() && plan_publisher_->get_subscription_count() > 0) {
    plan_publisher_->publish(std::move(msg));
  }
}


rcl_interfaces::msg::SetParametersResult
PlannerServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);
  rcl_interfaces::msg::SetParametersResult result;

  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {
      if (name == "expected_planner_frequency") {
        if (parameter.as_double() > 0) {
          max_planner_duration_ = 1 / parameter.as_double();
        } else {
          RCLCPP_WARN(
            get_logger(),
            "The expected planner frequency parameter is %.4f Hz. The value should to be greater"
            " than 0.0 to turn on duration overrrun warning messages", parameter.as_double());
          max_planner_duration_ = 0.0;
        }
      }
    }
  }

  result.successful = true;
  return result;
}

}  // namespace orunav2_planner

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(plan2_planner::PlannerServer)
