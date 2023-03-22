cmake_minimum_required(VERSION 3.8)
project(plan2_lifecycle_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(plan2_msgs REQUIRED)
find_package(plan2_util REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(bondcpp REQUIRED)
find_package(diagnostic_updater REQUIRED)

include_directories(
  include
)

set(library_name ${PROJECT_NAME}_core)

add_library(${library_name} SHARED
  src/lifecycle_manager.cpp
  src/lifecycle_manager_client.cpp
)

set(dependencies
  geometry_msgs
  lifecycle_msgs
  plan2_msgs
  plan2_util
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  std_msgs
  std_srvs
  tf2_geometry_msgs
  bondcpp
  diagnostic_updater
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

add_executable(task_lifecycle_manager
  src/main.cpp
)

target_link_libraries(task_lifecycle_manager
  ${library_name}
)

ament_target_dependencies(task_lifecycle_manager
  ${dependencies}
)

rclcpp_components_register_nodes(${library_name} "plan2_lifecycle_manager::LifecycleManager")

install(TARGETS
  ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  task_lifecycle_manager
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include/)

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
