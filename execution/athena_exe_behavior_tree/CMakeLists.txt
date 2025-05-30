cmake_minimum_required(VERSION 3.8)
project(athena_exe_behavior_tree)

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
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(athena_util REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(athena_msgs REQUIRED)
find_package(athena_exe_msgs REQUIRED)
find_package(material_handler_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(athena_behavior_tree REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(location_msgs REQUIRED)

include_directories(
  include
)

set(library_name ${PROJECT_NAME})

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  athena_msgs
  athena_exe_msgs
  material_handler_msgs
  geometry_msgs
  sensor_msgs
  nav2_msgs
  nav_msgs
  behaviortree_cpp
  tf2_ros
  tf2
  tf2_geometry_msgs
  std_msgs
  std_srvs
  athena_util
  visualization_msgs
  athena_behavior_tree
  nav2_costmap_2d
  location_msgs
)

#Actions
add_library(athena_exe_noop_action_bt_node SHARED plugins/action/noop_action.cpp)
list(APPEND plugin_libs athena_exe_noop_action_bt_node)

add_library(athena_exe_send_move_action_bt_node SHARED plugins/action/send_move_action.cpp)
list(APPEND plugin_libs athena_exe_send_move_action_bt_node)

add_library(athena_exe_send_transport_action_bt_node SHARED plugins/action/send_transport_action.cpp)
list(APPEND plugin_libs athena_exe_send_transport_action_bt_node)

add_library(athena_exe_send_load_action_bt_node SHARED plugins/action/send_load_action.cpp)
list(APPEND plugin_libs athena_exe_send_load_action_bt_node)

add_library(athena_exe_send_dump_action_bt_node SHARED plugins/action/send_dump_action.cpp)
list(APPEND plugin_libs athena_exe_send_dump_action_bt_node)
#Conditions
add_library(athena_exe_is_pile_moved_condition_bt_node SHARED plugins/condition/is_pile_moved_condition.cpp)
list(APPEND plugin_libs athena_exe_is_pile_moved_condition_bt_node)

#Controls
add_library(athena_exe_move_pile_method_node SHARED plugins/control/move_pile_method_node.cpp)
list(APPEND plugin_libs athena_exe_move_pile_method_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)


ament_export_include_directories(
  include
)

ament_export_libraries(
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
