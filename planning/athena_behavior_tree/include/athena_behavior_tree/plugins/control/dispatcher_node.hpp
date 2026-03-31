#ifndef ATHENA_BEHAVIOR_TREE__PLUGINS__DISPATCHER_NODE_HPP_
#define ATHENA_BEHAVIOR_TREE__PLUGINS__DISPATCHER_NODE_HPP_

#include <string>
#include "behaviortree_cpp/control_node.h"
#include "athena_msgs/msg/plan.hpp"
#include "rclcpp/rclcpp.hpp"
namespace athena_behavior_tree
{
/**

 */
class DispatcherNode : public BT::ControlNode
{
public:

    typedef std::vector<athena_msgs::msg::Action> Actions;
    typedef std::vector<athena_msgs::msg::Method> Methods;
    typedef std::vector<int> IDs;

    /**
     * @brief A constructor for athena_behavior_tree::DispatcherNode
     * @param name Name for the XML tag for this node
     * @param conf BT node configuration
     */
    DispatcherNode(
        const std::string & name,
        const BT::NodeConfiguration & conf);

    /**
     * @brief A destructor for athena_behavior_tree::DispatcherNode   
     */
    ~DispatcherNode() override = default;

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing basic ports along with node-specific ports
    */

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<athena_msgs::msg::Plan>("execution_plan", "The computed execution plan"),
            BT::InputPort<std::string>("type","actions", "If the plan is for actions or methods, should be either 'action' or 'method'"),
            BT::InputPort<IDs>("completed_methods","The set of completed methods"),
            BT::InputPort<IDs>("completed_actions","The set of completed actions"),
            BT::OutputPort<Methods>("concurrent_methods","The set of all methods that can be executed at current step"),
            BT::OutputPort<Actions>("concurrent_actions","The set of all actions that can be executed at current step"),
        };
    }

    private:
        unsigned int current_child_idx_;
        int plan_length_;
        int count_;
        std::map<std::string, Actions> plan_actions_;
        std::map<std::string, Methods> plan_methods_;
        athena_msgs::msg::Plan execution_plan_;
        std::string type_;
        rclcpp::Logger logger_ ;
        IDs completed_;
    

    /**
     * @brief The main override required by a BT action
     * @return BT::NodeStatus Status of tick execution
     */
    BT::NodeStatus tick() override;

    /**
     * @brief The other (optional) override required by a BT action to reset node state
     */
    void halt() override;

      /**
     * @brief Read the methods from the exection plan
     * 
     * @param execution_plan_ The execution plan
     * @return The set of methods defined in the exeuction plan
     */
    void readPlan();

    void dispatch();
};

} 

#endif 
