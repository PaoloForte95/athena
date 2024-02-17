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

#include <chrono>
#include <string>
#include <memory>
#include <cmath>
#include "behaviortree_cpp_v3/decorator_node.h"


#include "athena_behavior_tree/plugins/decorator/selected_robot_controller.hpp"

namespace athena_behavior_tree
{

SelectedRobotController::SelectedRobotController(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf),
  first_time_(false)
{
  getInput("robot_id", robot_id_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");


}

inline BT::NodeStatus SelectedRobotController::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    // Reset the starting position since we're starting a new iteration of
    // the distance controller (moving from IDLE to RUNNING)
    first_time_ = true;
  }

  setStatus(BT::NodeStatus::RUNNING);
  Methods methods;
  config().blackboard->get<Methods>("concurrent_methods", methods);
  for(auto method : methods){
    int robotID = method.robotid;

    // The child gets ticked the first time through and every time the threshold
    // distance is crossed. In addition, once the child begins to run, it is
    // ticked each time 'til completion
    if ((child_node_->status() == BT::NodeStatus::RUNNING) ||
      robotID == robot_id_)
    {
      if(first_time_){
          first_time_ = false;
      }else{
        RCLCPP_INFO(node_->get_logger(), "Executing method for robot %d!", robot_id_);
      }

      const BT::NodeStatus child_state = child_node_->executeTick();


      switch (child_state) {
        case BT::NodeStatus::RUNNING:
          return BT::NodeStatus::RUNNING;

        case BT::NodeStatus::SUCCESS:
          return BT::NodeStatus::SUCCESS;

        case BT::NodeStatus::FAILURE:
        default:
          return BT::NodeStatus::FAILURE;
      }
    }
  
  }
  return status();
}

}  // namespace athena_exe_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_behavior_tree::SelectedRobotController>("SelectedRobotController");
}

