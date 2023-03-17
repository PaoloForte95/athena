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

#include "gtest/gtest.h"
#include "plan2_planner/planners/metric_ff.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;



TEST(MetricFFTest, test_metric_ff_planner)
{
 rclcpp_lifecycle::LifecycleNode::SharedPtr node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("MetricFFTest");
  auto metricff = std::make_unique<plan2_planner::MetricFF>();


  metricff->configure(node, "test");
  metricff->activate();

  std::string domain = "src/planning2/plan2_protobuf/PDDL/Domain.pddl";
  std::string problem = "src/planning2/plan2_protobuf/PDDL/Problem.pddl";

  try {
    metricff->computeExecutionPlan(domain, problem);
  } catch (...) {
  }



  metricff->deactivate();
  metricff->cleanup();

  metricff.reset();

}