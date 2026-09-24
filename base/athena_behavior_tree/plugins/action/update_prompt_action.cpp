#include <memory>
#include <string>
#include <map>
#include <string>
#include <fstream>



#include "athena_behavior_tree/plugins/action/update_prompt_action.hpp"

namespace athena_behavior_tree
{

void appendMapToFile(const std::string &filename, const std::map<std::string, int> &objMap) {
    // Open the file in append mode.
    std::ofstream outFile(filename, std::ios::app);
    if (!outFile) {
        //std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }
    
    // Write the header line.
    outFile << "In this problem you have:" << std::endl;
    
    // Write each map entry in the specified format.
    for (const auto &entry : objMap) {
        outFile << "- " << entry.first << ": " << entry.second << std::endl;
    }
    
    // Optional: flush and close the file.
    outFile.close();
}

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
    getInput("failure_type", failure_type_);

    generateNewPrompt();
    setOutput("updated_prompt", previous_prompt_);
    return BT::NodeStatus::SUCCESS;
    
}


std::string UpdatePromptAction::generateNewPrompt(){
  if(failure_type_.find("ObjectDetection") != std::string::npos){
    RCLCPP_ERROR(node_->get_logger(), "Executing repromting");
    std::map<std::string,int> objects_mismatch;
    config().blackboard->get<std::map<std::string,int>>("objects_mismatch", objects_mismatch);
    appendMapToFile(previous_prompt_,objects_mismatch);
  }
    return previous_prompt_;
}

}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_behavior_tree::UpdatePromptAction>("UpdatePrompt");
}