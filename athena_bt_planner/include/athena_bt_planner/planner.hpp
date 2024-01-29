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

#ifndef ATHENA_BT_PLANNER__PLANNER_HPP_
#define ATHENA_BT_PLANNER__PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <mutex>

#include "athena_util/odometry_utils.hpp"
#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"
#include "athena_behavior_tree/bt_action_server.hpp"

namespace athena_bt_planner
{


/**
 * @class PlannerMuxer
 * @brief A class to control the state of the BT planner by allowing only a single
 * plugin to be processed at a time.
 */
class PlannerMuxer
{
public:
  /**
   * @brief A Planner Muxer constructor
   */
  PlannerMuxer()
  : current_planner_(std::string("")) {}

  /**
   * @brief Get the planner muxer state
   * @return bool If a planner is in progress
   */
  bool isPlanning()
  {
    std::scoped_lock l(mutex_);
    return !current_planner_.empty();
  }

  /**
   * @brief Start planning with a given planner
   * @param string Name of the planner to start
   */
  void startPlanning(const std::string & planner_name)
  {
    std::scoped_lock l(mutex_);
    if (!current_planner_.empty()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("PlannerMutex"),
        "Major error! Planner requested while another planner"
        " task is in progress! This likely occurred from an incorrect"
        "implementation of a planner plugin.");
    }
    current_planner_ = planner_name;
  }

  /**
   * @brief Stop planning with a given planner
   * @param string Name of the planner ending task
   */
  void stopPlanning(const std::string & planner_name)
  {
    std::scoped_lock l(mutex_);
    if (current_planner_ != planner_name) {
      RCLCPP_ERROR(
        rclcpp::get_logger("PlannerMutex"),
        "Major error! Planner stopped while another planner"
        " task is in progress! This likely occurred from an incorrect"
        "implementation of a planner plugin.");
    } else {
      current_planner_ = std::string("");
    }
  }

protected:
  std::string current_planner_;
  std::mutex mutex_;
};

/**
 * @class Planner
 * @brief Planner interface that acts as a base class for all BT-based Planner action's plugins
 */
template<class ActionT>
class Planner
{
public:
  using Ptr = std::shared_ptr<athena_bt_planner::Planner<ActionT>>;

  /**
   * @brief A Planner constructor
   */
  Planner()
  {
    plugin_muxer_ = nullptr;
  }

  /**
   * @brief Virtual destructor
   */
  virtual ~Planner() = default;

  /**
   * @brief Configuration to setup the planner's backend BT and actions
   * @param parent_node The ROS parent node to utilize
   * @param plugin_lib_names a vector of plugin shared libraries to load
   * @param plugin_muxer The muxing object to ensure only one planner
   * can be active at a time
   * @param odom_smoother Object to get current smoothed robot's speed
   * @return bool If successful
   */
  bool on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    const std::vector<std::string> & plugin_lib_names,
    athena_bt_planner::PlannerMuxer * plugin_muxer,
    std::shared_ptr<athena_util::OdomSmoother> odom_smoother)
  {
    auto node = parent_node.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    plugin_muxer_ = plugin_muxer;

    // get the default behavior tree for this planner
    std::string default_bt_xml_filename = getDefaultBTFilepath(parent_node);

    // Create the Behavior Tree Action Server for this planner
    bt_action_server_ = std::make_unique<athena_behavior_tree::BtActionServer<ActionT>>(
      node,
      getName(),
      plugin_lib_names,
      default_bt_xml_filename,
      std::bind(&Planner::onGoalReceived, this, std::placeholders::_1),
      std::bind(&Planner::onLoop, this),
      std::bind(&Planner::onPreempt, this, std::placeholders::_1),
      std::bind(&Planner::onCompletion, this, std::placeholders::_1, std::placeholders::_2));

    bool ok = true;
    if (!bt_action_server_->on_configure()) {
      ok = false;
    }

    BT::Blackboard::Ptr blackboard = bt_action_server_->getBlackboard();
    blackboard->set<bool>("initial_pose_received", false);  // NOLINT
    blackboard->set<int>("number_recoveries", 0);  // NOLINT
    blackboard->set<std::shared_ptr<athena_util::OdomSmoother>>("odom_smoother", odom_smoother);  // NOLINT

    return configure(parent_node, odom_smoother) && ok;
  }

  /**
   * @brief Activation of the planner's backend BT and actions
   * @return bool If successful
   */
  bool on_activate()
  {
    bool ok = true;

    if (!bt_action_server_->on_activate()) {
      ok = false;
    }

    return activate() && ok;
  }

  /**
   * @brief Deactivation of the planner's backend BT and actions
   * @return bool If successful
   */
  bool on_deactivate()
  {
    bool ok = true;
    if (!bt_action_server_->on_deactivate()) {
      ok = false;
    }

    return deactivate() && ok;
  }

  /**
   * @brief Cleanup a planner
   * @return bool If successful
   */
  bool on_cleanup()
  {
    bool ok = true;
    if (!bt_action_server_->on_cleanup()) {
      ok = false;
    }

    bt_action_server_.reset();

    return cleanup() && ok;
  }

  /**
   * @brief Get the action name of this planner to expose
   * @return string Name of action to expose
   */
  virtual std::string getName() = 0;

  virtual std::string getDefaultBTFilepath(rclcpp_lifecycle::LifecycleNode::WeakPtr node) = 0;

  /**
   * @brief Get the action server
   * @return Action server pointer
   */
  std::unique_ptr<athena_behavior_tree::BtActionServer<ActionT>> & getActionServer()
  {
    return bt_action_server_;
  }

protected:
  /**
   * @brief An intermediate goal reception function to mux planners.
   */
  bool onGoalReceived(typename ActionT::Goal::ConstSharedPtr goal)
  {
    if (plugin_muxer_->isPlanning()) {
      RCLCPP_ERROR(
        logger_,
        "Requested plan from %s while another planner is processing,"
        " rejecting request.", getName().c_str());
      return false;
    }

    bool goal_accepted = goalReceived(goal);

    if (goal_accepted) {
      plugin_muxer_->startPlanning(getName());
    }

    return goal_accepted;
  }

  /**
   * @brief An intermediate completion function to mux planners
   */
  void onCompletion(
    typename ActionT::Result::SharedPtr result,
    const athena_behavior_tree::BtStatus final_bt_status)
  {
    plugin_muxer_->stopPlanning(getName());
    goalCompleted(result, final_bt_status);
  }

  /**
   * @brief A callback to be called when a new goal is received by the BT action server
   * Can be used to check if goal is valid and put values on
   * the blackboard which depend on the received goal
   */
  virtual bool goalReceived(typename ActionT::Goal::ConstSharedPtr goal) = 0;

  /**
   * @brief A callback that defines execution that happens on one iteration through the BT
   * Can be used to publish action feedback
   */
  virtual void onLoop() = 0;

  /**
   * @brief A callback that is called when a preempt is requested
   */
  virtual void onPreempt(typename ActionT::Goal::ConstSharedPtr goal) = 0;

  /**
   * @brief A callback that is called when a the action is completed; Can fill in
   * action result message or indicate that this action is done.
   */
  virtual void goalCompleted(
    typename ActionT::Result::SharedPtr result,
    const athena_behavior_tree::BtStatus final_bt_status) = 0;

  /**
   * @param Method to configure resources.
   */
  virtual bool configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr /*node*/,
    std::shared_ptr<athena_util::OdomSmoother>/*odom_smoother*/)
  {
    return true;
  }

  /**
   * @brief Method to cleanup resources.
   */
  virtual bool cleanup() {return true;}

  /**
   * @brief Method to activate any threads involved in execution.
   */
  virtual bool activate() {return true;}

  /**
   * @brief Method to deactivate and any threads involved in execution.
   */
  virtual bool deactivate() {return true;}

  std::unique_ptr<athena_behavior_tree::BtActionServer<ActionT>> bt_action_server_;
  rclcpp::Logger logger_{rclcpp::get_logger("TaskPlanner")};
  rclcpp::Clock::SharedPtr clock_;
  PlannerMuxer * plugin_muxer_;
};

}  

#endif 
