#ifndef ATHENA_BT_PLANNER__BT_PLANNER_HPP_
#define ATHENA_BT_PLANNER__BT_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "athena_util/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "athena_bt_planner/planners/generate_actions.hpp"

namespace athena_bt_planner
{

/**
 * @class athena_bt_planner::BtPlanner
 * @brief An action server that uses behavior tree for task planning
 */
class BtPlanner : public athena_util::LifecycleNode
{
public:
  /**
   * @brief A constructor for athena_bt_planner::BtPlanner class
   * @param options Additional options to control creation of the node.
   */
  explicit BtPlanner(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  /**
   * @brief A destructor for athena_bt_planner::BtPlanner class
   */
  ~BtPlanner();

protected:
  /**
   * @brief Configures member variables
   *
   * Initializes action server for "GenerateTasks"; subscription to
   * "task_sub"; and builds behavior tree from xml file.
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  athena_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Activates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  athena_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Deactivates action server
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  athena_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Resets member variables
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  athena_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  /**
   * @brief Called when in shutdown state
   * @param state Reference to LifeCycle node state
   * @return SUCCESS or FAILURE
   */
  athena_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

  // To handle all the BT related execution
  std::unique_ptr<athena_bt_planner::Planner<athena_msgs::action::GenerateTasks>> planner_;
  athena_bt_planner::PlannerMuxer plugin_muxer_;

};

}  

#endif 
