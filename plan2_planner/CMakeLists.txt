cmake_minimum_required(VERSION 3.8)
project(plan2_planner)


add_compile_options(-std=c++17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(plan2_core REQUIRED)
find_package(plan2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(plan2_protobuf REQUIRED)
find_package(protobuf REQUIRED)


include_directories(
  include
  ${Protobuf_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)

set(dependencies
  rclcpp
  pluginlib
  ament_index_cpp
  rclcpp_components
  nav2_util
  plan2_core
  plan2_msgs
  plan2_protobuf
  protobuf
)

set(executable_name task_planner_server)

add_executable(${executable_name}
  src/main.cpp
)

set(library_name plan2_planner)

add_library(${library_name} SHARED
  src/task_planner_server.cpp
)
ament_target_dependencies(${library_name} ${dependencies})
rclcpp_components_register_nodes(${library_name} "plan2_planner::TaskPlannerServer")



add_library(${library_name}_metricff SHARED
  src/planners/metric_ff.cpp
)

ament_target_dependencies(${library_name}_metricff
  ${dependencies}
)

add_library(${library_name}_lpg SHARED
  src/planners/lpg.cpp
)

ament_target_dependencies(${library_name}_lpg
  ${dependencies}
)



ament_target_dependencies(${executable_name} ${dependencies})
target_link_libraries(${executable_name} ${library_name} ${library_name}_metricff ${library_name}_lpg)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS ${library_name} ${library_name}_metricff ${library_name}_lpg
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()


ament_export_include_directories(include)
ament_export_libraries(${library_name} ${library_name}_metricff ${library_name}_lpg)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(plan2_core planners_plugin.xml)

ament_package()
