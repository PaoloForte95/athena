#include <string>
#include <memory>
#include <yaml-cpp/yaml.h>

#include "athena_behavior_tree/plugins/condition/check_problem_file_condition.hpp"
namespace athena_behavior_tree
{

CheckProblemFileCondition::CheckProblemFileCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
    node_ = rclcpp::Node::make_shared("check_problem_file_node");   
}

BT::NodeStatus CheckProblemFileCondition::tick()
{
  if(checkProblemFile()){
    setOutput("isProblemDefined", true);
  }
    else{
        setOutput("isProblemDefined", false);
    }
  return BT::NodeStatus::SUCCESS;
}

bool CheckProblemFileCondition::checkProblemFile(){

    std::string problem_file_;
    getInput<std::string>("problem_file", problem_file_);
    if(problem_file_.empty()){
        RCLCPP_WARN(node_->get_logger(), "Problem file is not defined!");
        return false;
    }
    return true;
}


}  // namespace athena_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_behavior_tree::CheckProblemFileCondition>("CheckProblemFile");
}