# Test MetricFF
ament_add_gtest(test_metric_ff_planner
test_metric_ff_planner.cpp
)
ament_target_dependencies(test_metric_ff_planner
  ${dependencies} 
)

target_link_libraries(test_metric_ff_planner ${library_name}_metricff
)

# Test LPG
ament_add_gtest(test_lpg_planner
test_lpg_planner.cpp
)
ament_target_dependencies(test_lpg_planner
  ${dependencies} 
)

target_link_libraries(test_lpg_planner ${library_name}_lpg
)