#include <string>
#include "athena_behavior_tree/plugins/control/dispatcher_node.hpp"

namespace athena_behavior_tree
{

DispatcherNode::DispatcherNode(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode::ControlNode(name, conf),
  current_child_idx_(0),
  plan_length_(0),
  count_(0),
  logger_(rclcpp::get_logger("DispatcherNode"))
{

}

BT::NodeStatus DispatcherNode::tick()
{
    getInput("execution_plan", execution_plan_);
    getInput("type", type_);

    if(plan_length_ == 0){
        readPlan();
    }

    // Update completed count
    if(type_ == "action"){
        completed_ = getInput<IDs>("completed_actions").value_or(IDs());
    } else if (type_ == "method"){
        completed_ = getInput<IDs>("completed_methods").value_or(IDs());
    }
    count_ = completed_.size();

    // Remove completed actions/methods from the plan
    removeCompleted();

    if(count_ >= plan_length_){
        haltChildren();
        return BT::NodeStatus::SUCCESS;
    }

    dispatch();

    // Tick ALL children every tick
    for(unsigned i = 0; i < children_nodes_.size(); i++){
        BT::NodeStatus child_status = children_nodes_[i]->executeTick();
        if(child_status == BT::NodeStatus::FAILURE){
            haltChildren();
            return BT::NodeStatus::FAILURE;
        }
    }

    return BT::NodeStatus::RUNNING;
}

void DispatcherNode::halt()
{
    haltChildren();
    count_ = 0;
    current_child_idx_ = 0;
    plan_length_ = 0;
    plan_actions_.clear();
    plan_methods_.clear();
    completed_.clear();
}

void DispatcherNode::removeCompleted()
{
    for (auto& [robot, actions] : plan_actions_) {
        actions.erase(
            std::remove_if(actions.begin(), actions.end(),
                [this](const auto& action) {
                    return std::find(completed_.begin(), completed_.end(),
                        action.action_id) != completed_.end();
                }),
            actions.end());
    }
    for (auto& [robot, methods] : plan_methods_) {
        methods.erase(
            std::remove_if(methods.begin(), methods.end(),
                [this](const auto& method) {
                    return std::find(completed_.begin(), completed_.end(),
                        method.id) != completed_.end();
                }),
            methods.end());
    }
}

void DispatcherNode::readPlan(){
    std::vector<std::string> robotIDs;

    if(type_ == "action"){
        for(athena_msgs::msg::Action action : execution_plan_.actions){
            std::string robot = action.robot;
            plan_length_ += 1;
            auto itr = std::find(robotIDs.begin(), robotIDs.end(), robot);
            if (itr == robotIDs.end()){
                robotIDs.push_back(robot);
                auto robot_state = robot + "_state";
                config().blackboard->set<std::string>(robot_state, "free");
            }
            if(plan_actions_.find(robot) == plan_actions_.end()){
                auto actions = std::vector<athena_msgs::msg::Action>();
                actions.push_back(action);
                plan_actions_[robot] = actions;
            }
            else{
                Actions actions = plan_actions_[robot];
                actions.push_back(action);
                plan_actions_[robot] = actions;
            }
        }
    } 
    else if (type_ == "method"){
        for(athena_msgs::msg::Method method : execution_plan_.methods){
            std::string robot = method.robot;
            plan_length_ += 1;
            auto itr = std::find(robotIDs.begin(), robotIDs.end(), robot);
            if (itr == robotIDs.end()){
                robotIDs.push_back(robot);
                auto robot_state = robot + "_state";
                config().blackboard->set<std::string>(robot_state, "free");
            }
            if(plan_methods_.find(robot) == plan_methods_.end()){
                auto methods = std::vector<athena_msgs::msg::Method>();
                methods.push_back(method);
                plan_methods_[robot] = methods;
            }
            else{
                Methods methods = plan_methods_[robot];
                methods.push_back(method);
                plan_methods_[robot] = methods;
            }
        }
    }
    config().blackboard->set< std::vector<std::string>>("robot_ids", robotIDs);

}

void DispatcherNode::dispatch(){ 
    Actions concurrent_actions;
    Methods concurrent_methods;
    if(type_ == "action"){
        for (const auto& pair : plan_actions_){
            auto robot = pair.first;
            auto actions = pair.second;
            std::string robot_state;
            config().blackboard->get<std::string>(robot + "_state", robot_state);
            RCLCPP_INFO(logger_, "robot %s state %s!", robot.c_str(), robot_state.c_str());
            if(robot_state == "free"){
                if (actions.empty()){
                    continue;
                }
                auto curr_action = actions[0];
                bool action_can_be_executed = true;
                for(auto pa : curr_action.parents){
                    int parent_id = pa;
                    auto it_parent = std::find(completed_.begin(), completed_.end(), parent_id);
                    if(it_parent == completed_.end()){
                        action_can_be_executed = false;
                        RCLCPP_INFO(logger_, "Action %d cannot be execute because its parent %d has not been completed yet!", curr_action.action_id, pa);
                        break;
                    }
                }
                if(action_can_be_executed){
                    RCLCPP_INFO(logger_, "Sending action %d, %s....", curr_action.action_id, curr_action.name.c_str());
                    concurrent_actions.push_back(curr_action);
                    config().blackboard->set<std::string>(robot + "_state", "busy");
                }
            }
            else{
                RCLCPP_INFO(logger_, "robot %s is busy!", robot.c_str());
            }
        }
        RCLCPP_INFO(logger_, "Concurrent actions size %d", concurrent_actions.size());
        setOutput("concurrent_actions", concurrent_actions);
        config().blackboard->set<Actions>("concurrent_actions", concurrent_actions);
    } else if (type_ == "method"){
        for(const auto& pair : plan_methods_){
            auto robot = pair.first;
            std::string robot_state;
            config().blackboard->get<std::string>(robot + "_state", robot_state);
            RCLCPP_INFO(logger_, "robot %s state %s!", robot.c_str(), robot_state.c_str());
            
            if(robot_state == "free"){
                auto robot_methods = plan_methods_[robot];
                if(robot_methods.empty()){
                    continue;
                }
                auto curr_method = robot_methods[0];
                bool method_can_be_executed = true;
                for(auto pm : curr_method.parents){
                    int parent_id = pm;
                    auto it_parent = std::find(completed_.begin(), completed_.end(), parent_id);
                    if(it_parent == completed_.end()){
                        method_can_be_executed = false;
                        RCLCPP_INFO(logger_, "Method %d cannot be execute because its parent %d has not been completed yet!", curr_method.id, pm);
                        break;
                    }
                }
                if(method_can_be_executed){
                    RCLCPP_INFO(logger_, "Sending method %d, %s....", curr_method.id, curr_method.name.c_str());
                    concurrent_methods.push_back(curr_method);
                    config().blackboard->set<std::string>(robot + "_state", "busy");

                    // Resolve method subtasks into actions for children to use
                    Actions subtask_actions;
                    for (int subtask_id : curr_method.substasks) {
                        for (const auto& action : execution_plan_.actions) {
                            if (action.action_id == subtask_id) {
                                subtask_actions.push_back(action);
                                break;
                            }
                        }
                    }
                    config().blackboard->set<Actions>("concurrent_actions", subtask_actions);
                }
            }
            else{
                RCLCPP_INFO(logger_, "robot %s is busy!", robot.c_str());
            }
        }
        setOutput("concurrent_methods", concurrent_methods);
        config().blackboard->set<Methods>("concurrent_methods", concurrent_methods);
    }
}

}  // namespace athena_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<athena_behavior_tree::DispatcherNode>("DispatcherNode");
}