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

#ifndef ATHENA_CORE__PLANNER_HPP_
#define ATHENA_CORE__PLANNER_HPP_

#include <optional>
#include <string>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "athena_msgs/msg/plan.hpp"

namespace athena_core
{

class Planner
{
public:
  using Ptr = std::shared_ptr<Planner>;


  std::string property_filename_;
  std::string proto_filename_;


   /**
   * @brief Virtual destructor
   */
  virtual ~Planner() {}

    virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name) = 0;

    /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to active planner and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactive planner and any threads involved in execution.
   */
  virtual void deactivate() = 0;
  
  /**
   * @brief Compute an execution plan given the planning domain and problem files.
   * 
   * @param domain the planning domain file.
   * @param problem the planning problem file.
   * @return athena_msgs::msg::Plan An execution plan of the provided planning problem.
   */
  virtual athena_msgs::msg::Plan computeExecutionPlan(const std::string & domain, const std::string & problem) = 0;

  /**
   * @brief Check if the provided planning domain is valid.
   * 
   * @param problem 
   * @return true if valide, otherwise false
   */
  virtual bool validateDomain(const std::string & domain) = 0;

  /**
   * @brief Check if the provided planning problem is valid.
   * 
   * @param problem 
   * @return true if valide, otherwise false
   */
  virtual bool validateProblem(const std::string & problem) = 0;
};

}  

#endif  
