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
#include "athena_behavior_tree/plugins/control/execute_plan_node.hpp"

namespace athena_behavior_tree
{

ExecutePlanNode::ExecutePlanNode(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode::ControlNode(name, conf),
  current_child_idx_(0),
  plan_length_(1),
  actions_count_(0)
{

}

BT::NodeStatus ExecutePlanNode::tick()
{
  const unsigned children_count = children_nodes_.size();
  
  getInput("plan_length", plan_length_);

  if (children_count > 2) {
    throw BT::BehaviorTreeException("Execute Plan Node '" + name() + "' must have 2 children. The first need to be the dispatcher, the second the logic to execute the actions.");
  }

  setStatus(BT::NodeStatus::RUNNING);

  while (current_child_idx_ < children_count && actions_count_ <= plan_length_) {
    TreeNode * child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child_node->executeTick();

    if (current_child_idx_ == 0) {
      switch (child_status) {
        case BT::NodeStatus::SUCCESS:
          {
            // reset node and return SUCCESS when first child returns success (plan is completed)
            halt();
            return BT::NodeStatus::SUCCESS;

          }

        case BT::NodeStatus::FAILURE:
           if (actions_count_ < plan_length_) {
              // halt first child and tick second child in next iteration
              ControlNode::haltChild(0);
              current_child_idx_++;
              break;
            } else {
              // reset node and return failure when max retries has been exceeded
              halt();
              return BT::NodeStatus::FAILURE;
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

    } else if (current_child_idx_ == 1) {
      switch (child_status) {
        case BT::NodeStatus::SUCCESS:
          {
            // halt second child, and tick first child in next iteration
            ControlNode::haltChild(1);
            config().blackboard->get<int>("actions_count", actions_count_);
            current_child_idx_--;
          }
          break;

        case BT::NodeStatus::FAILURE:
          {
            // reset node and return failure if second child fails
            halt();
            return BT::NodeStatus::FAILURE;
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
    }
  }  // end while loop

  // reset node and return failure
  halt();
  return BT::NodeStatus::FAILURE;
}

void ExecutePlanNode::halt()
{
  ControlNode::halt();
  actions_count_ = 0;
  current_child_idx_ = 0;
}

}  // namespace athena_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_behavior_tree::ExecutePlanNode>("ExecutePlanNode");
}
