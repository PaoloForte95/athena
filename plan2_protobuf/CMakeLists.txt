cmake_minimum_required(VERSION 3.8)
project(plan2_protobuf)
add_compile_options(-std=c++17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Protobuf REQUIRED)
find_package(plan2_msgs REQUIRED)


set(library_name ${PROJECT_NAME}_core)

add_subdirectory(src/main/proto)

set(dependencies
Protobuf
plan2_msgs
)

include_directories(
  include
  ${Protobuf_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)


add_library(${library_name} SHARED
  src/execution_plan.cpp
  src/action.cpp
  src/method.cpp
)

ament_target_dependencies(${library_name} ${dependencies})
target_link_libraries(${library_name} proto ${Protobuf_LIBRARIES})


install(DIRECTORY include/
  DESTINATION include/
)

install(TARGETS
  ${library_name}
  proto
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
ament_export_libraries(${library_name} proto)
ament_export_dependencies(${dependencies})
ament_package()
