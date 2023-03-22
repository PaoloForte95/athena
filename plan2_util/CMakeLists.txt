cmake_minimum_required(VERSION 3.8)
project(plan2_util)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(plan2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bondcpp REQUIRED)
find_package(bond REQUIRED)
find_package(action_msgs REQUIRED)

set(dependencies
    plan2_msgs
    tf2_ros
    tf2
    tf2_geometry_msgs
    geometry_msgs
    nav_msgs
    rclcpp
    lifecycle_msgs
    rclcpp_action
    rclcpp_lifecycle
    bondcpp
    bond
    action_msgs
)

include_directories(include)

set(library_name ${PROJECT_NAME}_core)

add_library(${library_name} SHARED
  src/lifecycle_node.cpp
  src/node_utils.cpp
  src/string_utils.cpp
  src/lifecycle_utils.cpp
  src/node_thread.cpp
  src/lifecycle_service_client.cpp
  src/robot_utils.cpp
  src/odometry_utils.cpp
)

ament_target_dependencies(${library_name}
  rclcpp
  plan2_msgs
  tf2
  tf2_ros
  nav_msgs
  geometry_msgs
  lifecycle_msgs
  rclcpp_lifecycle
  tf2_geometry_msgs
  bondcpp
)

add_executable(lifecycle_bringup
  src/lifecycle_bringup_commandline.cpp
)
target_link_libraries(lifecycle_bringup ${library_name})

find_package(Boost REQUIRED COMPONENTS program_options)

install(TARGETS
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  lifecycle_bringup
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/
  DESTINATION include/
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

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})

ament_package()
