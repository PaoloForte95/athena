// Copyright (c) 2023 Paolo Forte
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef ATHENA_CE_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_MOVE_ACTION_HPP_
#define ATHENA_CE_BEHAVIOR_TREE__PLUGINS__ACTION__SEND_MOVE_ACTION_HPP_

#include <Eigen/Geometry>
#include <Eigen/Core>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "athena_behavior_tree/bt_action_node.hpp"
#include "athena_msgs/msg/action.hpp"
#include <location_msgs/srv/get_waypoint_list.hpp>
namespace athena_exe_behavior_tree
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
 * @brief Create an action client to send the current action of the execution plan.
 * 
 */


template<typename MoveT>
class SendMoveAction : public BT::ActionNodeBase
{
public:
  /**
   * @brief A constructor for athena_behavior_tree::SendMoveAction
   * @param action_name Action name this node creates a client for
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
    return 
      {
        BT::InputPort<std::string>("service_name", "The name of the service"), 
        BT::InputPort<std::string>("global_frame", "map","Global frame")
      };
  }

protected:
  typedef std::vector<athena_msgs::msg::Action> Actions;
  typedef typename rclcpp_action::Client<MoveT>::SharedPtr Client;
  typedef typename rclcpp_action::ClientGoalHandle<MoveT> GoalHandleSendMove;
  typedef typename GoalHandleSendMove::WrappedResult GoalHandleSendMoveWrappedResult;
  typedef typename GoalHandleSendMove::SharedPtr GoalHandleSendMovePtr;
  typedef typename MoveT::Feedback MoveFeedback;
  typedef typename MoveT::Goal MoveGoal;
  typedef typename rclcpp_action::Client<MoveT>::SendGoalOptions SendGoalOptions;
  void sendMove(Actions actions);
  Actions getMoveActions();



private:
  std::string service_name_, global_frame_;
  rclcpp::Client<location_msgs::srv::GetWaypointList>::SharedPtr waypoint_client_;
  Client client_ptr_;
  GoalHandleSendMovePtr send_move_handler_;
  rclcpp::Node::SharedPtr node_;
  ActionStatus action_status_;
  Actions actions_;
  std::map<std::string, Waypoint> waypoints_;
  MoveGoal goal_;
  SendGoalOptions send_goal_options_;


   void goal_response_callback(const GoalHandleSendMovePtr & goal_handle);

   void feedback_callback(GoalHandleSendMovePtr, const std::shared_ptr<const MoveFeedback> feedback);

   void result_callback(const GoalHandleSendMoveWrappedResult & result);
   
   void getGoalLocation(std::string goal_waypoint);
   
   void setGoalOption();

    Eigen::Quaterniond rpyToQuaternion(double roll, double pitch, double yaw) {
      Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
      Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
      Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
      Eigen::Quaterniond quaternion = yawAngle * pitchAngle * rollAngle;
    return quaternion;
}

}; //End Class


}  // namespace athena_behavior_tree

#endif 