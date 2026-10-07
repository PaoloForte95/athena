cmake_minimum_required(VERSION 3.8)
project(athena_state_updater)


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
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)


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
  cv_bridge
)

set(executable_name state_updater_server)

add_executable(${executable_name}
  src/main.cpp
)


set(library_name athena_state_updater)

add_library(${library_name} SHARED
  src/state_updater_server.cpp
)
ament_target_dependencies(${library_name} ${dependencies})
rclcpp_components_register_nodes(${library_name} "athena_state_updater::StateUpdaterServer")


#State Updaters

add_library(${library_name}_simple SHARED
  src/state_updaters/simple_state_updater.cpp
)

ament_target_dependencies(${library_name}_simple
  ${dependencies}
)


ament_target_dependencies(${executable_name} ${dependencies})
target_link_libraries(${executable_name} ${library_name}  ${library_name}_simple)



install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS ${library_name} ${library_name}_simple
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   set(ament_cmake_copyright_FOUND TRUE)
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
#   find_package(ament_cmake_gtest REQUIRED)
#   add_subdirectory(test)
# endif()


ament_export_include_directories(include)
ament_export_libraries(${library_name} ${library_name}_simple)
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(athena_core state_updaters_plugin.xml)

ament_package()
