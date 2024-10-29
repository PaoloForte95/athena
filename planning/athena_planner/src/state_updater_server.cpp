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

#include "athena_planner/state_updater_server.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace athena_planner
{

StateUpdaterServer::StateUpdaterServer(const rclcpp::NodeOptions & options)
: athena_util::LifecycleNode("state_updater_server", "", options),
  gp_loader_("athena_core", "athena_core::StateUpdater"),
  default_ids_{"SimpleStateUpdater"},
  default_types_{"athena_planner::SimpleUpdater"}
{
  RCLCPP_INFO(get_logger(), "Creating task planner server");

  declare_parameter("planner_frequency", 20.0);
  // Declare this node's parameters
  declare_parameter("planner_plugins", default_ids_);

}

StateUpdaterServer::~StateUpdaterServer()
{
  state_updaters_.clear();

}

athena_util::CallbackReturn
StateUpdaterServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  auto node = shared_from_this();
  RCLCPP_INFO(get_logger(), "Configuring task planner interface");


  get_parameter("planner_plugins", state_updater_ids_);
  if (state_updater_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      athena_util::declare_parameter_if_not_declared(
        node, default_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_ids_[i]));
    }
  }
  state_updater_types_.resize(state_updater_ids_.size());

  get_parameter("state_updater_frequency", state_updater_frequency_);
  RCLCPP_INFO(get_logger(), "State Updater frequency set to %.4fHz", state_updater_frequency_);


  state_updater_types_.resize(state_updater_ids_.size());


  for (size_t i = 0; i != state_updater_ids_.size(); i++) {
    try {
      state_updater_types_[i] = athena_util::get_plugin_type_param(
        node, state_updater_ids_[i]);
      athena_core::StateUpdater::Ptr state_updater =
        gp_loader_.createUniqueInstance(state_updater_types_[i]);
      RCLCPP_INFO(
        get_logger(), "Created state updater plugin %s of type %s",
        state_updater_ids_[i].c_str(), state_updater_types_[i].c_str());
      state_updater->configure(node, state_updater_ids_[i]);
      state_updaters_.insert({state_updater_ids_[i], state_updater});
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_FATAL(
        get_logger(), "Failed to create the state updater. Exception: %s",
        ex.what());
      return athena_util::CallbackReturn::FAILURE;
    }
  }

  for (size_t i = 0; i != state_updater_ids_.size(); i++) {
    state_updater_ids_concat_ += state_updater_ids_[i] + std::string(" ");
  }

  RCLCPP_INFO(
    get_logger(),
    "Task Planner Server has %s planners available.", state_updater_ids_concat_.c_str());

 
  // Initialize pubs & subs
  state_publisher_ = create_publisher<athena_msgs::msg::State>("planning_problem_state", 1);

  // Create the action servers for path planning to a pose and through poses
  action_server_update_ = std::make_unique<ActionServerUpdate>(
    shared_from_this(),
    "update_state",
    std::bind(&StateUpdaterServer::updateState, this),
    nullptr,
    std::chrono::milliseconds(500),
    true);

  return athena_util::CallbackReturn::SUCCESS;
}

athena_util::CallbackReturn
StateUpdaterServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  state_publisher_->on_activate();
  action_server_update_->activate();


  StateUpdaterMap::iterator it;
  for (it = state_updaters_.begin(); it != state_updaters_.end(); ++it) {
    it->second->activate();
  }

  auto node = shared_from_this();


  // Add callback for dynamic parameters
  dyn_params_handler_ = node->add_on_set_parameters_callback(
    std::bind(&StateUpdaterServer::dynamicParametersCallback, this, _1));

  // create bond connection
  createBond();

  return athena_util::CallbackReturn::SUCCESS;
}

athena_util::CallbackReturn
StateUpdaterServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  action_server_update_->deactivate();
  state_publisher_->on_deactivate();

  StateUpdaterMap::iterator it;
  for (it = state_updaters_.begin(); it != state_updaters_.end(); ++it) {
    it->second->deactivate();
  }

  dyn_params_handler_.reset();

  // destroy bond connection
  destroyBond();

  return athena_util::CallbackReturn::SUCCESS;
}

athena_util::CallbackReturn
StateUpdaterServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  action_server_update_.reset();
  state_publisher_.reset();


  StateUpdaterMap::iterator it;
  for (it = state_updaters_.begin(); it != state_updaters_.end(); ++it) {
    it->second->cleanup();
  }
  state_updaters_.clear();

  return athena_util::CallbackReturn::SUCCESS;
}

athena_util::CallbackReturn
StateUpdaterServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return athena_util::CallbackReturn::SUCCESS;
}

template<typename T>
bool StateUpdaterServer::isServerInactive(
  std::unique_ptr<athena_util::SimpleActionServer<T>> & action_server)
{
  if (action_server == nullptr || !action_server->is_server_active()) {
    RCLCPP_DEBUG(get_logger(), "Action server unavailable or inactive. Stopping.");
    return true;
  }

  return false;
}



template<typename T>
bool StateUpdaterServer::isCancelRequested(
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
void StateUpdaterServer::getPreemptedGoalIfRequested(
  std::unique_ptr<athena_util::SimpleActionServer<T>> & action_server,
  typename std::shared_ptr<const typename T::Goal> goal)
{
  if (action_server->is_preempt_requested()) {
    goal = action_server->accept_pending_goal();
  }
}


void
StateUpdaterServer::updateState()
{
  std::lock_guard<std::mutex> lock(dynamic_params_lock_);

  auto start_time = steady_clock_.now();

  // Initialize the ComputePathToPose goal and result
  auto goal = action_server_update_->get_current_goal();
  auto result = std::make_shared<ActionUpdate::Result>();

  try {
    if (isServerInactive(action_server_update_) || isCancelRequested(action_server_update_)) {
      return;
    }

    getPreemptedGoalIfRequested(action_server_update_, goal);

    RCLCPP_INFO( get_logger(), "Updating the state with: %s ", goal->state_updater.c_str());

    result->updated_state = getUpdatedState(goal->previous_state, goal->actions,  goal->state_updater);
    auto message = athena_msgs::msg::State();
    message = result->updated_state;
    // Publish the plan for visualization purposes
    publishState(message);

    action_server_update_->succeeded_current(result);
  } catch (std::exception & ex) {
    RCLCPP_WARN(get_logger(), "%s plugin failed to update the state!",goal->state_updater.c_str());
    action_server_update_->terminate_current();
  }
}

athena_msgs::msg::State StateUpdaterServer::getUpdatedState(
    const athena_msgs::msg::State & previous_state,
    const Actions & actions,
    const std::string & state_updater)
{
   RCLCPP_INFO(get_logger(), "Attempting to update the state using state updater %s\"",state_updater.c_str());
    for (auto s : previous_state.state){
      RCLCPP_INFO(get_logger(), "Prev state %s",s.c_str());
    }
    athena_msgs::msg::State state;
      if (state_updaters_.find(state_updater) != state_updaters_.end()) {

      return state_updaters_[state_updater]->updateState(actions,previous_state);
    } else {
    if (state_updaters_.size() == 1 && state_updater.empty()) {
      RCLCPP_WARN_ONCE(
        get_logger(), "No state updater specified in action call. "
        "Server will use only plugin %s in server."
        " This warning will appear once.", state_updater_ids_concat_.c_str());
      return state_updaters_[state_updaters_.begin()->first]->updateState(actions, previous_state);
    } else {
      RCLCPP_ERROR(
        get_logger(), "state updater %s is not a valid state updater. "
        "State Updater Planner are: %s", state_updater.c_str(),
        state_updater_ids_concat_.c_str());
    }
  }

  return athena_msgs::msg::State();
}

void
StateUpdaterServer::publishState(const athena_msgs::msg::State & msg)
{
  auto message = std::make_unique<athena_msgs::msg::State>(msg);
  if (state_publisher_->is_activated() && state_publisher_->get_subscription_count() > 0) {
    state_publisher_->publish(std::move(message));
  }
}


rcl_interfaces::msg::SetParametersResult
StateUpdaterServer::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters)
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
RCLCPP_COMPONENTS_REGISTER_NODE(athena_planner::StateUpdaterServer)
