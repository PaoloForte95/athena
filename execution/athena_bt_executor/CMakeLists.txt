cmake_minimum_required(VERSION 3.8)
project(athena_bt_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(athena_util REQUIRED)
find_package(athena_core REQUIRED)
find_package(athena_msgs REQUIRED)
find_package(athena_behavior_tree REQUIRED)
find_package(athena_bt_planner REQUIRED)


include_directories(
  include
)


set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  rclcpp_components
  athena_msgs
  athena_util
  athena_core
  athena_behavior_tree
  behaviortree_cpp
  athena_bt_planner
  std_msgs
)

set(executable_name bt_executor)
set(executable_name_action bt_action_executor)
set(library_name ${executable_name}_core)


add_executable(${executable_name} src/main.cpp
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

add_executable(${executable_name_action} src/main_action.cpp
)

ament_target_dependencies(${executable_name_action}
  ${dependencies}
)

add_library(${library_name} SHARED
  src/bt_executor.cpp
  src/method_executor.cpp
  src/action_executor.cpp
  src/bt_action_executor.cpp
)

ament_target_dependencies(${library_name}
  ${dependencies}
)

target_link_libraries(${executable_name} ${library_name})
target_link_libraries(${executable_name_action} ${library_name})


rclcpp_components_register_nodes(${library_name} "athena::execution::BtExecutor")
rclcpp_components_register_nodes(${library_name} "athena::execution::BtActionExecutor")

install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${executable_name_action}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
   set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${library_name})
ament_export_dependencies(${dependencies})


ament_package()
