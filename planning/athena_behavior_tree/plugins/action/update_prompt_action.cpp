#include <memory>
#include <string>

#include "athena_behavior_tree/plugins/action/update_prompt_action.hpp"

namespace athena_behavior_tree
{

UpdatePromptAction::UpdatePromptAction(
  const std::string & action_name,
  const BT::NodeConfiguration & conf)
: ActionNodeBase(action_name, conf)
{
    node_ = rclcpp::Node::make_shared("Update_prompt_node");   
}


inline BT::NodeStatus UpdatePromptAction::tick()
{
    setStatus(BT::NodeStatus::RUNNING);
    //Get the computed PDDL problem instance
    getInput("previous_prompt", previous_prompt_);


    auto new_prompt = generateNewPrompt();
    setOutput("updated_prompt", new_prompt);
    return BT::NodeStatus::SUCCESS;
    
}




std::string UpdatePromptAction::generateNewPrompt(){
    return previous_prompt_;

}

}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_behavior_tree::UpdatePromptAction>("UpdatePrompt");
}