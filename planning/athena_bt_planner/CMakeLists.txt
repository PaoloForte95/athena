cmake_minimum_required(VERSION 3.8)
project(athena_bt_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(athena_util REQUIRED)
find_package(athena_core REQUIRED)
find_package(athena_msgs REQUIRED)
find_package(athena_behavior_tree REQUIRED)


include_directories(
  include
)


set(executable_name bt_planner)

add_executable(${executable_name} src/main.cpp
)

set(library_name ${executable_name}_core)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  geometry_msgs
  nav_msgs
  athena_msgs
  athena_util
  athena_core
  athena_behavior_tree
  behaviortree_cpp
  std_srvs
  tf2_ros
  nav2_common
)

add_library(${library_name} SHARED
  src/bt_planner.cpp
  src/planners/task_planner.cpp
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})

ament_target_dependencies(${library_name}
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "athena_bt_planner::BtPlanner")

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
