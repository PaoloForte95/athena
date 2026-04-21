#ifndef ATHENA_BEHAVIOR_TREE__PLUGINS__CONTROL__MOVE_PILE_METHOD_NODE_HPP_
#define ATHENA_BEHAVIOR_TREE__PLUGINS__CONTROL__MOVE_PILE_METHOD_NODE_HPP_

#include <string>
#include <vector>
#include "behaviortree_cpp/control_node.h"
#include "athena_msgs/msg/method.hpp"
#include "athena_msgs/msg/action.hpp"
#include "athena_msgs/msg/plan.hpp"
namespace athena_behavior_tree
{

using Methods = std::vector<athena_msgs::msg::Method>;
using Actions = std::vector<athena_msgs::msg::Action>;
using IDs = std::vector<int>;

class MovePileMethodNode : public BT::ControlNode
{
public:
  MovePileMethodNode(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  ~MovePileMethodNode() override = default;

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("robot_id", "Robot ID assigned to this node"),
    };
  }

private:
  unsigned int current_child_idx_;
  int active_method_id_{-1};
  Actions subtask_actions_;

  BT::NodeStatus tick() override;
  void halt() override;
};

}  // namespace athena_behavior_tree

#endif