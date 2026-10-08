#include <string>
#include <vector>
#include <utility>

#include "athena_state_updater/state_updaters/scene_graph_state_updater.hpp"

namespace athena_state_updater
{

SceneGraphStateUpdater::SceneGraphStateUpdater() {}

SceneGraphStateUpdater::~SceneGraphStateUpdater()
{
  RCLCPP_INFO(logger_, "Destroying plugin %s of type SceneGraphStateUpdater", name_.c_str());
}

void SceneGraphStateUpdater::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name)
{
  node_ = parent;
  auto node = parent.lock();
  logger_ = node->get_logger();
  name_ = name;

  const std::string topic_param = name_ + ".scene_graph_topic";
  if (!node->has_parameter(topic_param)) {
    node->declare_parameter(topic_param, std::string("scene_graph"));
  }
  node->get_parameter(topic_param, scene_graph_topic_);

  scene_graph_sub_ = node->create_subscription<scene_graph_msgs::msg::SceneGraph>(
    scene_graph_topic_,
    rclcpp::QoS(1).transient_local().reliable(),
    [this](scene_graph_msgs::msg::SceneGraph msg) {
      sceneGraphCallback(msg);
    });

  RCLCPP_INFO(
    logger_, "Configured plugin %s of type SceneGraphStateUpdater, reading %s",
    name_.c_str(), scene_graph_topic_.c_str());
}

void SceneGraphStateUpdater::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s of type SceneGraphStateUpdater", name_.c_str());

  standard_msgs::msg::StringMultiArray state;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_graph_) {
      return;
    }
    state.data = facts_;
  }
  reportState(state);
}

void SceneGraphStateUpdater::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type SceneGraphStateUpdater", name_.c_str());
}

void SceneGraphStateUpdater::cleanup()
{
  RCLCPP_INFO(logger_, "Cleaning up plugin %s of type SceneGraphStateUpdater", name_.c_str());
  scene_graph_sub_.reset();
}

standard_msgs::msg::StringMultiArray SceneGraphStateUpdater::updateState(
  const std::vector<standard_msgs::msg::Action> &,
  const standard_msgs::msg::StringMultiArray & previous_state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!has_graph_) {
    return previous_state;
  }

  standard_msgs::msg::StringMultiArray state;
  state.data = facts_;
  return state;
}

void SceneGraphStateUpdater::sceneGraphCallback(const scene_graph_msgs::msg::SceneGraph & msg)
{
  std::vector<std::string> facts;
  facts.reserve(msg.edges.size());
  for (const auto & edge : msg.edges) {
    std::string fact = "(" + edge.relation;
    for (const auto & arg : edge.args) {
      fact += " " + arg;
    }
    fact += ")";
    facts.push_back(fact);
  }

  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (has_graph_ && facts == facts_) {
      return;
    }
    facts_ = facts;
    has_graph_ = true;
  }

  RCLCPP_INFO(logger_, "Scene graph changed, the state now has %zu facts", facts.size());

  standard_msgs::msg::StringMultiArray state;
  state.data = facts;
  reportState(state);
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(athena_state_updater::SceneGraphStateUpdater, athena_core::StateUpdater)
