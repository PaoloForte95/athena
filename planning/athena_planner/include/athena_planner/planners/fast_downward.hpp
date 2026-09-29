// Copyright (c) 2026 Paolo Forte
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

#ifndef ATHENA_PLANNER__PLANNERS__FAST_DOWNWARD_HPP_
#define ATHENA_PLANNER__PLANNERS__FAST_DOWNWARD_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "athena_planning_core/planner.hpp"
#include "athena_msgs/msg/plan.hpp"
#include "athena_util/lifecycle_node.hpp"

namespace athena_planner
{

class FastDownward : public athena_planning_core::Planner{

public:
    /**
     * @brief Construct a new Fast Downward object
     *
     */
    FastDownward();


   /**
    * @brief Destroy the Fast Downward object
    *
    */
  ~FastDownward();

  /**
   * @brief Configuring plugin
   * @param parent Lifecycle node pointer
   * @param name Name of plugin
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
   * @brief Compute an execution plan given the planning domain and problem files.
   *
   * @param domain the planning domain file.
   * @param problem the planning problem file.
   * @return athena_msgs::msg::Plan An execution plan of the provided planning problem.
   */
  athena_msgs::msg::Plan computeExecutionPlan(const std::string & domain, const std::string & problem) override;


  /**
   * @brief Check if the provided planning domain is valid.
   *
   * @param domain
   * @return true if valid, otherwise false
   */
  bool validateDomain(const std::string & domain) override;

   /**
   * @brief Check if the provided planning problem is valid.
   *
   * @param problem
   * @return true if valid, otherwise false
   */
  bool validateProblem(const std::string & problem) override;


  /**
   * @brief Callback executed when a parameter change is detected
   * @param parameters Changed parameters
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

protected:

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;
  rclcpp::Logger logger_{rclcpp::get_logger("FastDownward")};

  // Search configuration passed to Fast Downward, e.g. "astar(blind())" or "lama-first"
  std::string search_{"astar(blind())"};

  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _dyn_params_handler;

}; //Class FastDownward

}


#endif
