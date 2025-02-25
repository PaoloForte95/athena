cmake_minimum_required(VERSION 3.8)
project(athena_msgs)

add_compile_options(-std=c++17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME} 
   #Msgs
   "msg/Action.msg"
   "msg/BehaviorTreeLog.msg"
   "msg/BehaviorTreeStatusChange.msg"
   "msg/Execution.msg"
   "msg/Method.msg"
   "msg/Plan.msg"
   "msg/PlanningProblem.msg"
   "msg/State.msg"
   #Services
   "srv/GetPlan.srv"
   "srv/RecomputePlan.srv"
   "srv/ManageLifecycleNodes.srv"
   "srv/GenerateProblemFile.srv"
   "srv/DetectObjects.srv"
   #Actions
   "action/ComputePlan.action"
   "action/OptmizePlan.action"
   "action/ExecuteMethod.action"
   "action/ExecuteAction.action"
   "action/UpdateState.action"
  DEPENDENCIES std_msgs action_msgs builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
