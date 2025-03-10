#include <string>
#include <memory>
#include <yaml-cpp/yaml.h>

#include "athena_behavior_tree/plugins/condition/all_object_detected_condition.hpp"
namespace athena_behavior_tree
{

AllObjectDetectedCondition::AllObjectDetectedCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
    node_ = rclcpp::Node::make_shared("all_object_detected_node");   
}

BT::NodeStatus AllObjectDetectedCondition::tick()
{
  if(checkListObjects()){
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

bool AllObjectDetectedCondition::checkListObjects(){
    bool list_is_correct = true;

    YAML::Node obj_file = YAML::LoadFile("/home/pofe/cuda_shared/dev_ws/object_tally.yaml");
    RCLCPP_DEBUG(node_->get_logger(), "Checking list of objects...");
    std::map<std::string,int> objects_mismatch;
    for (const auto& node : obj_file) {
        std::string object_name = node.first.as<std::string>();  // Get object name (e.g., "mug")
        YAML::Node obj = node.second;  // Access the object's data

        auto val1 = obj["LLM"].as<int>();
        auto val2 = obj["SAM"].as<int>();

        if (val1 != val2){
            list_is_correct = false;
            objects_mismatch.insert(std::pair<std::string, int>(object_name, val2));
            RCLCPP_ERROR(node_->get_logger(), "Object %s mismatch! LLM: %d  SAM: %d!", object_name.c_str(), val1, val2);           
        }
    }
    config().blackboard->set<std::map<std::string,int>>("objects_mismatch", objects_mismatch);
    RCLCPP_DEBUG(node_->get_logger(), "All objects detected correctly!");

    return list_is_correct;
}


}  // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_behavior_tree::AllObjectDetectedCondition>("AllObjectDetected");
}