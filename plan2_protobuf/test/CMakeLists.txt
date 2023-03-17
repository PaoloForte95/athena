# Test Read Plan File 
ament_add_gtest(plan_reader
plan_reader.cpp
)

target_link_libraries(plan_reader proto ${Protobuf_LIBRARIES} ${library_name})
ament_target_dependencies(plan_reader ${dependencies})
