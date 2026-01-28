# Test Read Plan File 

find_package(absl REQUIRED)
ament_add_gtest(plan_reader
plan_reader.cpp
)

target_link_libraries(plan_reader 
proto 
absl::log
absl::check
absl::base
absl::strings
${Protobuf_LIBRARIES} 
${library_name})

ament_target_dependencies(plan_reader ${dependencies})
