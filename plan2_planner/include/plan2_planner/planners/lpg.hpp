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

#ifndef PLAN2_PLANNER__PLANNERS__LPG_HPP_
#define PLAN2_PLANNER__PLANNERS__LPG_HPP_

#include "plan2_core/planner.hpp"
#include "plan2_msgs/msg/plan.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace plan2_planner
{
class LPG : public plan2_core::Planner
{

public:
    /**
     * @brief Construct a new Metric FF object
     * 
     */
   LPG();


   /**
    * @brief Destroy the Metric FF object
    * 
    */
  ~LPG();

  /**
   * @brief Configuring plugin
   * @param parent Lifecycle node pointer
   * @param name Name of plugin map
   * @param tf Shared ptr of TF2 buffer
   * @param costmap_ros Costmap2DROS object
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
   * @return plan2_msgs::msg::Plan An execution plan of the provided planning problem.
   */
  plan2_msgs::msg::Plan computeExecutionPlan(const std::string & domain, const std::string & problem) override;
  

  /**
   * @brief Check if the provided planning domain is valid.
   * 
   * @param problem 
   * @return true if valide, otherwise false
   */
  bool validateDomain(const std::string & domain) override;
   
   /**
   * @brief Check if the provided planning problem is valid.
   * 
   * @param problem 
   * @return true if valide, otherwise false
   */
  bool validateProblem(const std::string & problem) override;

  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);


protected:

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_, proto_filename_, output_filename_;
  rclcpp::Logger logger_{rclcpp::get_logger("LPGTD")};

  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _dyn_params_handler;

}; //Class LPG


} // namespace plan2_planner


#endif  // PLAN2_PLANNER__PLANNERS__LPG_HPP_