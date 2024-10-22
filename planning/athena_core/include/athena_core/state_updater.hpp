// Copyright 2023 Paolo Forte
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

#ifndef ATHENA_CORE__STATE_UPDATER_HPP_
#define ATHENA_CORE__STATE_UPDATER_HPP_

#include <optional>
#include <string>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "athena_msgs/msg/state.hpp"
#include "athena_msgs/msg/action.hpp"

namespace athena_core
{

class StateUpdater
{
public:
  using Ptr = std::shared_ptr<StateUpdater>;

   /**
   * @brief Virtual destructor
   */
  virtual ~StateUpdater() {}

    virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name) = 0;

    /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to active state updater and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactive state updater and any threads involved in execution.
   */
  virtual void deactivate() = 0;
  
  /**
   * @brief Update the planning state
   * 
   * @param domain 
   * @param problem 
   * @return athena_msgs::msg::State 
   */
  virtual athena_msgs::msg::State updateState(const std::vector<athena_msgs::msg::Action> & actions, const athena_msgs::msg::State& previous_state) = 0;

};

}  

#endif  
