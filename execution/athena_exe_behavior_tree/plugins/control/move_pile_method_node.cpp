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

#include <string>
#include "athena_exe_behavior_tree/plugins/control/move_pile_method_node.hpp"
#include "rclcpp/rclcpp.hpp"
namespace athena_exe_behavior_tree
{

MovePileMethodNode::MovePileMethodNode(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode::ControlNode(name, conf),
  current_child_idx_(0)
{

}

BT::NodeStatus MovePileMethodNode::tick()
{
  const unsigned children_count = children_nodes_.size();
  auto node = rclcpp::Node::make_shared("move_pile_method_node");
  if (children_count != 5) {
    throw BT::BehaviorTreeException("Move Pile Method Node '" + name() + "' must only have 5 children. The first four corresponds to the actions: move, load, move, dump. The last one is a check for the amount of material to move.");
  }
  
  setStatus(BT::NodeStatus::RUNNING);
  while (current_child_idx_ < children_count ) {
    TreeNode * child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child_node->executeTick();
    //Check last child
    if (current_child_idx_ == 4) {
      switch (child_status) {
        case BT::NodeStatus::SUCCESS:
          {    
            // reset node and return SUCCESS when last child returns SUCCESS
            halt();
            return BT::NodeStatus::SUCCESS;
          }

        case BT::NodeStatus::FAILURE:
          {
            // reset node and re-ticks previous children if the last child returns false
            halt();
            return BT::NodeStatus::RUNNING;
          }

        case BT::NodeStatus::RUNNING:
          {
            return BT::NodeStatus::RUNNING;
          }

        default:
          {
            throw BT::LogicError("A child node must never return IDLE");
          }
      }  // end switch

    } else if (current_child_idx_ < 4) {
      switch (child_status) {
          case BT::NodeStatus::FAILURE:
          {
            // reset node and return failure if any of the first four children return failure.
            halt();
            return BT::NodeStatus::FAILURE;
          }
        case BT::NodeStatus::SUCCESS:
            // do nothing and continue on to the next child.
            current_child_idx_ ++;
             break;
        case BT::NodeStatus::RUNNING:
          {
            return BT::NodeStatus::RUNNING;
          }
        
        default:
          {
            throw BT::LogicError("A child node must never return IDLE");
          }
      }  // end switch
    }
  }  // end while loop
  // reset node and return failure
  halt();
  return BT::NodeStatus::FAILURE;
}

void MovePileMethodNode::halt()
{
  ControlNode::halt();
  current_child_idx_ = 0;
}

}  // namespace athena_exe_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_exe_behavior_tree::MovePileMethodNode>("MovePileMethodNode");
}
