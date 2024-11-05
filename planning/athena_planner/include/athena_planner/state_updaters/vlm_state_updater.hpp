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

#ifndef ATHENA_PLANNER__PLANNERS__VLM_STATE_UPDATER_HPP_
#define ATHENA_PLANNER__PLANNERS__VLM_STATE_UPDATER_HPP_

#include "athena_core/state_updater.hpp"
#include "athena_msgs/msg/state.hpp"
#include "athena_util/lifecycle_node.hpp"
#include <sensor_msgs/msg/image.hpp>
namespace athena_planner
{
class VlmStateUpdater : public athena_core::StateUpdater
{

public:
    /**
     * @brief Construct a new Vlm State Updater object
     * 
     */
   VlmStateUpdater();


    /**
     * @brief Destroy the Vlm State Updater object
     * 
     */
  ~VlmStateUpdater();

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
   * @return athena_msgs::msg::Plan An execution plan of the provided planning problem.
   */
  athena_msgs::msg::State updateState(const std::vector<athena_msgs::msg::Action> & actions, const athena_msgs::msg::State& previous_state) override;
  
  /**
   * @brief Callback executed when a parameter change is detected
   * @param event ParameterEvent message
   */
  rcl_interfaces::msg::SetParametersResult dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

protected:
  
  /**
   * @brief Camera data callback
   * @param msg Shared pointer to Image message
   */
  void cameraCallback(sensor_msgs::msg::Image msg);

protected:

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_, camera_topic_;
  rclcpp::Logger logger_{rclcpp::get_logger("VlmStateUpdater")};

  // Dynamic parameters handler
  std::mutex mutex_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr _dyn_params_handler;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;

}; 


} 


#endif  //