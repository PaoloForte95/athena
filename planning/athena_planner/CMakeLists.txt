cmake_minimum_required(VERSION 3.8)
project(athena_planner)


add_compile_options(-std=c++17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(athena_core REQUIRED)
find_package(athena_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(athena_util REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(athena_protobuf REQUIRED)
find_package(Protobuf REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)


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
  athena_core
  athena_msgs
  sensor_msgs
  athena_util
  athena_protobuf
  Protobuf
)

set(executable_name task_planner_server)

add_executable(${executable_name}
  src/main.cpp
)

set(state_updater_executable_name state_updater_server)

add_executable(${state_updater_executable_name}
  src/main_state_updater.cpp
)

set(library_name athena_planner)
set(updater_library_name athena_updater)

add_library(${library_name} SHARED
  src/task_planner_server.cpp
)
ament_target_dependencies(${library_name} ${dependencies})
rclcpp_components_register_nodes(${library_name} "athena_planner::TaskPlannerServer")


add_library(${updater_library_name} SHARED
  src/state_updater_server.cpp
)
ament_target_dependencies(${updater_library_name} ${dependencies})
rclcpp_components_register_nodes(${updater_library_name} "athena_planner::StateUpdaterServer")

#Planners
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

add_library(${library_name}_tfd SHARED
  src/planners/tfd.cpp
)

ament_target_dependencies(${library_name}_tfd
  ${dependencies}
)

add_library(${library_name}_lilotane SHARED
  src/planners/lilotane.cpp
)

ament_target_dependencies(${library_name}_lilotane
  ${dependencies}
)

#State Updaters

add_library(${updater_library_name}_simple SHARED
  src/state_updaters/simple_state_updater.cpp
)

ament_target_dependencies(${updater_library_name}_simple
  ${dependencies}
)

add_library(${updater_library_name}_vlm SHARED
  src/state_updaters/vlm_state_updater.cpp
)

ament_target_dependencies(${updater_library_name}_vlm
  ${dependencies}
  OpenCV
  cv_bridge
)


ament_target_dependencies(${executable_name} ${dependencies})
target_link_libraries(${executable_name} ${library_name}  ${library_name}_metricff ${library_name}_lpg ${library_name}_tfd ${library_name}_lilotane)

ament_target_dependencies(${state_updater_executable_name} ${dependencies})
target_link_libraries(${state_updater_executable_name} ${updater_library_name} ${updater_library_name}_simple ${updater_library_name}_vlm)


install(TARGETS ${executable_name} ${state_updater_executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS ${library_name} ${library_name}_metricff ${library_name}_lpg ${library_name}_tfd ${library_name}_lilotane
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${updater_library_name} ${updater_library_name}_simple ${updater_library_name}_vlm
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
ament_export_libraries(${library_name}${library_name}_metricff ${library_name}_lpg ${library_name}_tfd ${library_name}_lilotane)
ament_export_libraries(${updater_library_name} ${updater_library_name}_simple ${updater_library_name}_vlm)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(athena_core planners_plugin.xml)
pluginlib_export_plugin_description_file(athena_core state_updaters_plugin.xml)

ament_package()
