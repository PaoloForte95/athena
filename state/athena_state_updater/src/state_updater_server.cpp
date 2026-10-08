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
#include <algorithm>
#include <cctype>

#include "builtin_interfaces/msg/duration.hpp"
#include "athena_util/node_utils.hpp"
#include "athena_util/geometry_utils.hpp"

#include "athena_state_updater/state_updater_server.hpp"

using namespace std::chrono_literals;
using rcl_interfaces::msg::ParameterType;
using std::placeholders::_1;

namespace athena_state_updater
{

namespace
{

struct SExpr
{
  std::string atom;
  std::vector<SExpr> children;
  bool isAtom() const {return !atom.empty();}
};

std::string toLower(const std::string & text)
{
  std::string result = text;
  std::transform(
    result.begin(), result.end(), result.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});
  return result;
}

std::vector<std::string> lexPddl(const std::string & text)
{
  std::vector<std::string> tokens;
  std::string current;
  bool in_comment = false;

  auto flush = [&]() {
      if (!current.empty()) {
        tokens.push_back(current);
        current.clear();
      }
    };

  for (char c : text) {
    if (in_comment) {
      if (c == '\n') {
        in_comment = false;
      }
      continue;
    }
    if (c == ';') {
      flush();
      in_comment = true;
    } else if (c == '(' || c == ')') {
      flush();
      tokens.push_back(std::string(1, c));
    } else if (std::isspace(static_cast<unsigned char>(c))) {
      flush();
    } else {
      current += c;
    }
  }
  flush();
  return tokens;
}

SExpr parseSExpr(const std::vector<std::string> & tokens, size_t & pos)
{
  SExpr expr;
  if (pos >= tokens.size()) {
    return expr;
  }
  const std::string token = tokens[pos++];
  if (token != "(") {
    expr.atom = token;
    return expr;
  }
  while (pos < tokens.size() && tokens[pos] != ")") {
    expr.children.push_back(parseSExpr(tokens, pos));
  }
  ++pos;
  return expr;
}

std::string toString(const SExpr & expr)
{
  if (expr.isAtom()) {
    return expr.atom;
  }
  std::string text = "(";
  for (size_t i = 0; i < expr.children.size(); ++i) {
    if (i > 0) {
      text += " ";
    }
    text += toString(expr.children[i]);
  }
  return text + ")";
}

std::vector<std::string> parseInitialState(const std::string & problem)
{
  std::vector<std::string> facts;
  auto tokens = lexPddl(problem);
  size_t pos = 0;
  auto root = parseSExpr(tokens, pos);

  for (const auto & section : root.children) {
    if (section.children.empty() || !section.children[0].isAtom()) {
      continue;
    }
    if (toLower(section.children[0].atom) != ":init") {
      continue;
    }
    for (size_t i = 1; i < section.children.size(); ++i) {
      facts.push_back(toString(section.children[i]));
    }
  }
  return facts;
}

}

StateUpdaterServer::StateUpdaterServer(const rclcpp::NodeOptions & options)
: athena_util::LifecycleNode("state_updater_server", "", options),
  gp_loader_("athena_core", "athena_core::StateUpdater"),
  default_ids_{"SimpleStateUpdater"},
  default_types_{"athena_state_updater::SimpleStateUpdater"}
{
  RCLCPP_INFO(get_logger(), "Creating state updater server");

  declare_parameter("frequency", 20.0);
  // Declare this node's parameters
  declare_parameter("plugins", default_ids_);

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


  get_parameter("plugins", state_updater_ids_);
  if (state_updater_ids_ == default_ids_) {
    for (size_t i = 0; i < default_ids_.size(); ++i) {
      athena_util::declare_parameter_if_not_declared(
        node, default_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_types_[i]));
    }
  }
  state_updater_types_.resize(state_updater_ids_.size());

  get_parameter("frequency", state_updater_frequency_);
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
    "State Updater Server has %s state updaters available.", state_updater_ids_concat_.c_str());

  if (state_updater_ids_.empty()) {
    RCLCPP_FATAL(get_logger(), "No state updater plugin is loaded");
    return athena_util::CallbackReturn::FAILURE;
  }

  if (state_updater_ids_.size() > 1) {
    RCLCPP_WARN(
      get_logger(), "More than one state updater plugin is loaded, only %s is used for action events",
      state_updater_ids_.front().c_str());
  }

  event_state_updater_ = state_updater_ids_.front();

  RCLCPP_INFO(get_logger(), "The state is updated with %s", event_state_updater_.c_str());

  state_updaters_[event_state_updater_]->setStateCallback(
    [this](const standard_msgs::msg::StringMultiArray & state) {
      onPluginState(state);
    });

 
  // Initialize pubs & subs
  state_publisher_ = create_publisher<standard_msgs::msg::StringMultiArray>(
    "/planning_state",
    rclcpp::QoS(1).transient_local().reliable());

  problem_sub_ = create_subscription<std_msgs::msg::String>(
    "planning_problem",
    rclcpp::QoS(1).transient_local().reliable(),
    [this](std_msgs::msg::String msg) {
      problemCallback(msg);
    });

  plan_sub_ = create_subscription<standard_msgs::msg::Plan>(
    "/dispatched_plan",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    [this](standard_msgs::msg::Plan msg) {
      planCallback(msg);
    });

  event_sub_ = create_subscription<standard_msgs::msg::Event>(
    "/plan_actions",
    rclcpp::QoS(rclcpp::KeepLast(1000)).transient_local().reliable(),
    [this](standard_msgs::msg::Event msg) {
      eventCallback(msg);
    });

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

  standard_msgs::msg::StringMultiArray stored_state;
  bool has_state = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    has_state = state_received_;
    stored_state = current_state_;
  }
  if (has_state) {
    publishState(stored_state);
  }


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
  plan_sub_.reset();
  event_sub_.reset();
  problem_sub_.reset();


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
    {
      std::lock_guard<std::mutex> state_lock(state_mutex_);
      current_state_ = result->updated_state;
      state_received_ = true;
    }
    auto message = standard_msgs::msg::StringMultiArray();
    message = result->updated_state;
    // Publish the plan for visualization purposes
    publishState(message);

    action_server_update_->succeeded_current(result);
  } catch (std::exception & ex) {
    RCLCPP_WARN(get_logger(), "%s plugin failed to update the state!",goal->state_updater.c_str());
    action_server_update_->terminate_current();
  }
}

void
StateUpdaterServer::planCallback(const standard_msgs::msg::Plan & msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (msg == plan_) {
    return;
  }
  plan_ = msg;
  applied_actions_.clear();
  RCLCPP_INFO(get_logger(), "Received plan with %zu actions", plan_.actions.size());
}

void
StateUpdaterServer::eventCallback(const standard_msgs::msg::Event & msg)
{
  if (msg.kind != standard_msgs::msg::Event::ACTION) {
    return;
  }

  if (msg.status == standard_msgs::msg::Event::FAILURE) {
    RCLCPP_WARN(get_logger(), "Action %d (%s) failed, effects not applied", msg.id, msg.name.c_str());
    return;
  }

  if (msg.status != standard_msgs::msg::Event::SUCCESS) {
    return;
  }

  standard_msgs::msg::StringMultiArray state;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (applied_actions_.count(msg.id) > 0) {
      return;
    }

    auto it = std::find_if(
      plan_.actions.begin(), plan_.actions.end(),
      [&](const standard_msgs::msg::Action & a) {return a.action_id == msg.id;});

    if (it == plan_.actions.end()) {
      RCLCPP_WARN(get_logger(), "Action %d (%s) not found in the plan", msg.id, msg.name.c_str());
      return;
    }

    current_state_ = getUpdatedState(current_state_, Actions{*it}, event_state_updater_);
    applied_actions_.insert(msg.id);
    state_received_ = true;
    state = current_state_;
  }

  RCLCPP_INFO(get_logger(), "Applied the effects of action %d (%s)", msg.id, msg.name.c_str());
  publishState(state);
}

void
StateUpdaterServer::problemCallback(const std_msgs::msg::String & msg)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    applied_actions_.clear();
    if (state_from_plugin_) {
      RCLCPP_INFO(
        get_logger(), "The state comes from plugin %s, the :init of the planning problem is not used",
        event_state_updater_.c_str());
      return;
    }
  }

  auto facts = parseInitialState(msg.data);
  if (facts.empty()) {
    RCLCPP_WARN(get_logger(), "No :init facts found in the planning problem, initial state not set");
    return;
  }

  standard_msgs::msg::StringMultiArray state;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_state_.data = facts;
    state_received_ = true;
    state = current_state_;
  }

  RCLCPP_INFO(
    get_logger(), "Initial state set with %zu facts from the planning problem",
    facts.size());
  publishState(state);
}

void
StateUpdaterServer::onPluginState(const standard_msgs::msg::StringMultiArray & state)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    current_state_ = state;
    state_received_ = true;
    state_from_plugin_ = true;
  }
  publishState(state);
}

standard_msgs::msg::StringMultiArray StateUpdaterServer::getUpdatedState(
    const standard_msgs::msg::StringMultiArray & previous_state,
    const Actions & actions,
    const std::string & state_updater)
{
   RCLCPP_INFO(get_logger(), "Attempting to update the state using state updater %s\"",state_updater.c_str());
    //for (auto s : previous_state.state){
      //RCLCPP_INFO(get_logger(), "Prev state %s",s.c_str());
    //}
    standard_msgs::msg::StringMultiArray state;
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

  return standard_msgs::msg::StringMultiArray();
}

void
StateUpdaterServer::publishState(const standard_msgs::msg::StringMultiArray & msg)
{
  if (state_publisher_ && state_publisher_->is_activated()) {
    state_publisher_->publish(msg);
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

}  // namespace athena_state_updater

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(athena_state_updater::StateUpdaterServer)