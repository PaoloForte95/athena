#ifndef ATHENA_PLANNING_CORE__STATE_UPDATER_HPP_
#define ATHENA_PLANNING_CORE__STATE_UPDATER_HPP_

#include <functional>
#include <optional>
#include <string>
#include <memory>
#include <utility>
#include <vector>


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "standard_msgs/msg/string_multi_array.hpp"
#include "standard_msgs/msg/action.hpp"

namespace athena_core
{

class StateUpdater
{
public:
  using Ptr = std::shared_ptr<StateUpdater>;
  using StateCallback = std::function<void (const standard_msgs::msg::StringMultiArray &)>;

   /**
   * @brief Virtual destructor
   */
  virtual ~StateUpdater() {}

    virtual void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name) = 0;

    /**
   * @brief Method to cleanup resources used on shutdown.
   */
  virtual void cleanup() = 0;

  /**
   * @brief Method to active state updater and any threads involved in execution.
   */
  virtual void activate() = 0;

  /**
   * @brief Method to deactive state updater and any threads involved in execution.
   */
  virtual void deactivate() = 0;
  
  /**
   * @brief Update the planning state
   * 
   * @param domain 
   * @param problem 
   * @param previous_state The current planning state
   * @return standard_msgs::msg::StringMultiArray 
   */
  virtual standard_msgs::msg::StringMultiArray updateState(const std::vector<standard_msgs::msg::Action> & actions, const standard_msgs::msg::StringMultiArray& previous_state) = 0;

  /**
   * @brief Set the function the plugin calls to report a new state
   * 
   * @param callback Function that receives the new state
   */
  void setStateCallback(StateCallback callback) {state_callback_ = std::move(callback);}

protected:
  /**
   * @brief Report a new state, if a function was set
   * 
   * @param state The new planning state
   */
  void reportState(const standard_msgs::msg::StringMultiArray & state)
  {
    if (state_callback_) {
      state_callback_(state);
    }
  }

  StateCallback state_callback_;
};

}  

#endif