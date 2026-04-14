#include <string>
#include "athena_behavior_tree/plugins/control/move_pile_method_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace athena_behavior_tree
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
  constexpr unsigned num_children = 5;

  if (children_count != num_children) {
    throw BT::BehaviorTreeException(
      "MovePileMethodNode '" + name() + "' must only have " +
      std::to_string(num_children) + " children.");
  }

  std::string robot_id;
  if (!getInput("robot_id", robot_id)) {
    throw BT::RuntimeError("MovePileMethodNode missing required input [robot_id]");
  }

  // Only check for a new method if we're not already executing one
  if (active_method_id_ == -1) {
    auto concurrent_methods = config().blackboard->get<Methods>("concurrent_methods");
    for (const auto& method : concurrent_methods) {
      if (method.robot == robot_id && method.name == "move_pile") {
        active_method_id_ = method.id;
        break;
      }
    }

    if (active_method_id_ == -1) {
      return BT::NodeStatus::SUCCESS;
    }
  }

  setStatus(BT::NodeStatus::RUNNING);
  while (current_child_idx_ < children_count) {
    TreeNode* child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child_node->executeTick();

    if (current_child_idx_ == (children_count - 1)) {
      switch (child_status) {
        case BT::NodeStatus::SUCCESS:
        {
          IDs completed_methods;
          config().blackboard->get<IDs>("completed_methods", completed_methods);
          completed_methods.push_back(active_method_id_);
          config().blackboard->set<IDs>("completed_methods", completed_methods);
          config().blackboard->set<std::string>(robot_id + "_state", "free");
          halt();
          return BT::NodeStatus::SUCCESS;
        }
        case BT::NodeStatus::FAILURE:
        {
          config().blackboard->set<std::string>(robot_id + "_state", "free");
          halt();
          return BT::NodeStatus::RUNNING;
        }
        case BT::NodeStatus::RUNNING:
          return BT::NodeStatus::RUNNING;
        default:
          throw BT::LogicError("A child node must never return IDLE");
      }
    } else {
      switch (child_status) {
        case BT::NodeStatus::FAILURE:
        {
          halt();
          return BT::NodeStatus::FAILURE;
        }
        case BT::NodeStatus::SUCCESS:
          current_child_idx_++;
          break;
        case BT::NodeStatus::RUNNING:
          return BT::NodeStatus::RUNNING;
        default:
          throw BT::LogicError("A child node must never return IDLE");
      }
    }
  }

  halt();
  return BT::NodeStatus::FAILURE;
}

void MovePileMethodNode::halt()
{
  ControlNode::halt();
  current_child_idx_ = 0;
  active_method_id_ = -1;
}

}  // namespace athena_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_behavior_tree::MovePileMethodNode>("MovePileMethodNode");
}