#ifndef ATHENA_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_MOVE_ACTION_HPP_
#define ATHENA_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_MOVE_ACTION_HPP_

#include <Eigen/Geometry>
#include <Eigen/Core>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "standard_msgs/action/move_to_pose.hpp"
#include "athena_behavior_tree/bt_action_node.hpp"
#include "athena_msgs/msg/action.hpp"

namespace athena_behavior_tree
{

enum class ActionStatus
{
  UNKNOWN = 0,
  PROCESSING = 1,
  FAILED = 2,
  SUCCEEDED = 3
};

struct Waypoint {
  double x;
  double y;
  double theta;
};

/**
 * @brief BT action node that sends a MoveToPose action goal for the
 *        current move action in the execution plan.
 */
class SendMoveAction : public BT::ActionNodeBase
{
public:
  using MoveToPose = standard_msgs::action::MoveToPose;
  using GoalHandleMoveToPose = rclcpp_action::ClientGoalHandle<MoveToPose>;

  /**
   * @brief A constructor for athena_exe_behavior_tree::SendMoveAction
   * @param service_name Service name this node creates a client for
   * @param conf BT node configuration
   */
  SendMoveAction(
    const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief The other (optional) override required by a BT action.
   */
  void halt() override {}

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("service_name", "The name of the MoveToPose action server"),
      BT::InputPort<std::string>("robot_id", "The name of the robot")
    };
  }

protected:
  typedef std::vector<athena_msgs::msg::Action> Actions;

  bool sendMove(Actions actions);
  Actions getMoveActions();

private:
  std::string service_name_, robot_id_;
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<MoveToPose>::SharedPtr client_ptr_;
  GoalHandleMoveToPose::SharedPtr goal_handle_;
  ActionStatus action_status_;
  Actions actions_;
  std::map<std::string, Waypoint> waypoints_;

  void goal_response_callback(const GoalHandleMoveToPose::SharedPtr & goal_handle);
  void result_callback(const GoalHandleMoveToPose::WrappedResult & result);

  MoveToPose::Goal buildGoal(const std::string & wp_name);

  Eigen::Quaterniond rpyToQuaternion(double roll, double pitch, double yaw) {
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    return yawAngle * pitchAngle * rollAngle;
  }
};

}

#endif