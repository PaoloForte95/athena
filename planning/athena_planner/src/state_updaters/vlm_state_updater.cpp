// Copyright (c) 2024 Paolo Forte
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
#include <regex>
#include "athena_util/node_utils.hpp"
#include "athena_planner/state_updaters/vlm_state_updater.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"


using std::placeholders::_1;
using rcl_interfaces::msg::ParameterType;

namespace athena_planner
{


VlmStateUpdater::VlmStateUpdater() {}


VlmStateUpdater::~VlmStateUpdater()
{
  RCLCPP_INFO(logger_, "Destroying plugin %s of type VlmStateUpdater", name_.c_str());
}


void VlmStateUpdater::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name)
{
  node_ = parent;
  auto node = parent.lock();
  logger_ = node->get_logger();
  name_ = name;

  athena_util::declare_parameter_if_not_declared(node, name + ".camera_topic", rclcpp::ParameterValue("/camera/camera/color/image_raw"));
  node->get_parameter<std::string>(name + ".camera_topic", camera_topic_);
  athena_util::declare_parameter_if_not_declared(node, name + ".image_filename", rclcpp::ParameterValue("image.png"));
  node->get_parameter<std::string>(name + ".image_filename", image_filename_);
  
  
  RCLCPP_INFO(logger_, "Configuring %s of type VlmStateUpdater", name.c_str());
  rclcpp::QoS scan_qos = rclcpp::SensorDataQoS(); 
  camera_sub_ = node->create_subscription<sensor_msgs::msg::Image>(camera_topic_, scan_qos,std::bind(&VlmStateUpdater::cameraCallback, this, std::placeholders::_1));
  

  RCLCPP_INFO( logger_, "Configured plugin %s of type CostmapSelector with ", name_.c_str());

}


void VlmStateUpdater::activate()
{
  RCLCPP_INFO(logger_, "Activating plugin %s of type VlmStateUpdater",name_.c_str());

  auto node = node_.lock();
  // Add callback for dynamic parameters
  _dyn_params_handler = node->add_on_set_parameters_callback(std::bind(&VlmStateUpdater::dynamicParametersCallback, this, _1));
}

void VlmStateUpdater::deactivate()
{
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type VlmStateUpdater",
    name_.c_str());
  _dyn_params_handler.reset();
}

void VlmStateUpdater::cleanup()
{
  RCLCPP_INFO(
    logger_, "Cleaning up plugin %s of type VlmStateUpdater",name_.c_str());

}

athena_msgs::msg::State VlmStateUpdater::updateState(const std::vector<athena_msgs::msg::Action> & actions, const athena_msgs::msg::State& previous_state){
    auto new_state = athena_msgs::msg::State();
    //Make the call to the vlm model in python to get the new state from camera image
    if (last_image_.data.empty()){
       RCLCPP_ERROR(logger_, "no image available!");
    }

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(last_image_, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR(logger_, "cv_bridge exception: %s", e.what());
        return new_state;
    }

    // Save image to a file
    if (cv::imwrite(image_filename_, cv_ptr->image)) {
        RCLCPP_INFO(logger_, "Image saved as %s", image_filename_.c_str());
    } else {
        RCLCPP_ERROR(logger_, "Failed to save image");
    }
    

    return new_state;

}

void VlmStateUpdater::cameraCallback(sensor_msgs::msg::Image msg){
  last_image_ = msg;
}


rcl_interfaces::msg::SetParametersResult VlmStateUpdater::dynamicParametersCallback(
  std::vector<rclcpp::Parameter> parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  std::lock_guard<std::mutex> lock_reinit(mutex_);

  bool reinit_downsampler = false;


  for (auto parameter : parameters) {
    const auto & type = parameter.get_type();
    const auto & name = parameter.get_name();

    if (type == ParameterType::PARAMETER_DOUBLE) {


    } else if (type == ParameterType::PARAMETER_BOOL) {

    }
    else if (type == ParameterType::PARAMETER_INTEGER){

    }
    else if (type == ParameterType::PARAMETER_STRING){

    
    }
  }

  result.successful = true;
  return result;
}


} //namespace athena_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(athena_planner::VlmStateUpdater, athena_core::StateUpdater)