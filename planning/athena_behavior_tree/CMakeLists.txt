cmake_minimum_required(VERSION 3.8)
project(athena_behavior_tree)

add_compile_options(-std=c++17)
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(athena_util REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(athena_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  athena_msgs
  geometry_msgs
  sensor_msgs
  nav2_msgs
  nav_msgs
  behaviortree_cpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  std_srvs
  athena_util
  visualization_msgs
  yaml-cpp
)

add_library(${library_name} SHARED
  src/behavior_tree_engine.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

#Actions
add_library(athena_compute_plan_action_bt_node SHARED plugins/action/compute_plan_action.cpp)
list(APPEND plugin_libs athena_compute_plan_action_bt_node)

add_library(athena_dispatch_tasks_action_bt_node SHARED plugins/action/dispatch_tasks_action.cpp)
list(APPEND plugin_libs athena_dispatch_tasks_action_bt_node)

add_library(athena_dispatch_methods_action_bt_node SHARED plugins/action/dispatch_methods_action.cpp)
list(APPEND plugin_libs athena_dispatch_methods_action_bt_node)

add_library(athena_send_method_action_bt_node SHARED plugins/action/send_method.cpp)
list(APPEND plugin_libs athena_send_method_action_bt_node)

add_library(athena_send_action_action_bt_node SHARED plugins/action/send_action.cpp)
list(APPEND plugin_libs athena_send_action_action_bt_node)

add_library(athena_update_state_action_bt_node SHARED plugins/action/update_state_action.cpp)
list(APPEND plugin_libs athena_update_state_action_bt_node)

add_library(athena_detect_objects_action_bt_node SHARED plugins/action/detect_objects_action.cpp)
list(APPEND plugin_libs athena_detect_objects_action_bt_node)



add_library(athena_generate_problem_file_action_bt_node SHARED plugins/action/generate_problem_file_action.cpp)
list(APPEND plugin_libs athena_generate_problem_file_action_bt_node)

add_library(athena_update_prompt_action_bt_node SHARED plugins/action/update_prompt_action.cpp)
list(APPEND plugin_libs athena_update_prompt_action_bt_node)

#Condition

add_library(athena_all_object_detected_condition_node SHARED plugins/condition/all_object_detected_condition.cpp)
list(APPEND plugin_libs athena_all_object_detected_condition_node)
target_link_libraries(athena_all_object_detected_condition_node yaml-cpp)



#Control
add_library(athena_execute_plan_node_bt_node SHARED plugins/control/execute_plan_node.cpp)
list(APPEND plugin_libs athena_execute_plan_node_bt_node)

add_library(athena_recovery_node_bt_node SHARED plugins/control/recovery_node.cpp)
list(APPEND plugin_libs athena_recovery_node_bt_node)


add_library(athena_pipeline_sequence_bt_node SHARED plugins/control/pipeline_sequence.cpp)
list(APPEND plugin_libs athena_pipeline_sequence_bt_node)

#Decorator
add_library(athena_selected_robot_controller_bt_node SHARED plugins/decorator/selected_robot_controller.cpp)
list(APPEND plugin_libs athena_selected_robot_controller_bt_node)

add_library(athena_rate_controller_bt_node SHARED plugins/decorator/rate_controller.cpp)
list(APPEND plugin_libs athena_rate_controller_bt_node)

add_library(athena_updated_prompt_controller_bt_node SHARED plugins/decorator/updated_prompt_controller.cpp)
list(APPEND plugin_libs athena_updated_prompt_controller_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${library_name} ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES behavior_trees/planning_tree_nodes.xml DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${library_name}
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})

install(DIRECTORY
  behavior_trees
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
