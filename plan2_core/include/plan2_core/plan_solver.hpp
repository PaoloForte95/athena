// Copyright 2022 Paolo Forte
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

#ifndef PLAN2_CORE__PLANSOLVER_HPP_
#define PLAN2_CORE__PLANSOLVER_HPP_

#include <optional>
#include <string>
#include <memory>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "plan2_msgs/msg/plan.hpp"

namespace plan2_core
{

class PlanSolver
{
public:
  using Ptr = std::shared_ptr<PlanSolver>;

   /**
   * @brief Virtual destructor
   */
  virtual ~PlanSolver() {}

  virtual void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr &, const std::string &) {}

  virtual plan2_msgs::msg::Plan getExecutionPlan(
    const std::string & domain, const std::string & problem, const std::string & node_namespace = "") = 0;
};

}  // namespace planning2_core

#endif  // PLAN2_CORE__PLANSOLVER_HPP_
