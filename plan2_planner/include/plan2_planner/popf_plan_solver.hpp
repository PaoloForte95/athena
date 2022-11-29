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

#ifndef PLAN2_POPF_PLAN_SOLVER_HPP_
#define PLAN2_POPF_PLAN_SOLVER_HPP_

#include <optional>
#include <memory>
#include <string>

#include "plan2_core/plan_solver.hpp"
#include "plan2_msgs/msg/plan.hpp"

namespace plan2_planner
{

class POPFPlanSolver : public plan2_core::PlanSolver
{
private:
  std::string parameter_name_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node_;

public:
  POPFPlanSolver();

  void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr &, const std::string &) override;


  plan2_msgs::msg::Plan getExecutionPlan(
    const std::string & domain, const std::string & problem,
    const std::string & node_namespace = "") override;

  bool is_valid_domain(
    const std::string & domain,
    const std::string & node_namespace = "");
};

}  // namespace plan2_planner

#endif  // PLAN2_POPF_PLAN_SOLVER_HPP_
