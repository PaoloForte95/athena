#ifndef ATHENA_STATE_UPDATER__STATE_UPDATERS__SCENE_GRAPH_STATE_UPDATER_HPP_
#define ATHENA_STATE_UPDATER__STATE_UPDATERS__SCENE_GRAPH_STATE_UPDATER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "athena_core/state_updater.hpp"
#include "scene_graph_msgs/msg/scene_graph.hpp"
#include "standard_msgs/msg/string_multi_array.hpp"

namespace athena_state_updater
{

class SceneGraphStateUpdater : public athena_core::StateUpdater
{
public:
  /**
   * @brief Construct a new Scene Graph State Updater object
   */
  SceneGraphStateUpdater();

  /**
   * @brief Destroy the Scene Graph State Updater object
   */
  ~SceneGraphStateUpdater();

  /**
   * @brief Configuring plugin
   * @param parent Lifecycle node pointer
   * @param name Name of the plugin
   */
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name) override;

  /**
   * @brief Cleanup lifecycle node
   */
  void cleanup() override;

  /**
   * @brief Activate lifecycle node
   */
  void activate() override;

  /**
   * @brief Deactivate lifecycle node
   */
  void deactivate() override;

  /**
   * @brief Return the facts of the latest scene graph
   * 
   * @param actions not used, the effects are already in the scene graph
   * @param previous_state returned when no scene graph has arrived yet
   * @return standard_msgs::msg::StringMultiArray the facts of the latest scene graph
   */
  standard_msgs::msg::StringMultiArray updateState(
    const std::vector<standard_msgs::msg::Action> & actions,
    const standard_msgs::msg::StringMultiArray & previous_state) override;

protected:
  /**
   * @brief Turn the edges of the scene graph into PDDL facts and report them when they change
   * @param msg The scene graph
   */
  void sceneGraphCallback(const scene_graph_msgs::msg::SceneGraph & msg);

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("SceneGraphStateUpdater")};
  std::string scene_graph_topic_;
  rclcpp::Subscription<scene_graph_msgs::msg::SceneGraph>::SharedPtr scene_graph_sub_;

  std::mutex mutex_;
  bool has_graph_{false};
  std::vector<std::string> facts_;
};

}

#endif
