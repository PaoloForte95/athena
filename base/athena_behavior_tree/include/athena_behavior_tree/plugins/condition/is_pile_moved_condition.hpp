#ifndef ATHENA_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PILE_MOVED_HPP_
#define ATHENA_BEHAVIOR_TREE__PLUGINS__CONDITION__IS_PILE_MOVED_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "material_handler_msgs/srv/get_material_amount.hpp"

namespace athena_behavior_tree
{

/**
 * @brief A BT::ConditionNode that check if the pile of material has been completely (or partially) moved from the loading position.
 */
class IsPileMovedCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for athena_behavior_tree::IsPileMovedCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsPileMovedCondition(const std::string & condition_name, const BT::NodeConfiguration & conf);

  IsPileMovedCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("criteria", "NumberCycles", "Criteria to use to check if the pile is completely moved."),
      BT::InputPort<std::string>("image_topic", "image","The camera image topic"),
      BT::InputPort<int>("ticks", 1, "Number of ticks to do (If NumberCycles criteria is selected)"),
      BT::InputPort<double>("desired_amount",0.0,"The desired amount of material to move"),
      BT::InputPort<std::string>("robot_id", "The name of the robot")
    };
  }


private: 



private:

  rclcpp::Node::SharedPtr node_;
  std::string global_frame_, image_topic_, criteria_, robot_id_;
  int ticks_, current_tick_;
  double desired_amount_;
  rclcpp::Client<material_handler_msgs::srv::GetMaterialAmount>::SharedPtr material_amount_client_;
};

}

#endif
