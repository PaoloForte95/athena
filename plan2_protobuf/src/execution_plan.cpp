
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>


#include "plan2_protobuf/execution_plan.hpp"

using namespace std;

namespace planning2{

std::vector<plan2_msgs::msg::Action> Plan::GetActions(const planning2::ExecutionPlan& execution_plan) {
  std::vector<plan2_msgs::msg::Action> actions;
  for (int i = 0; i < execution_plan.action_size(); i++) {
    plan2_msgs::msg::Action act;
    act.name = execution_plan.action(i).name();
    act.material = execution_plan.action(i).material();
    act.action_id = execution_plan.action(i).id();
    act.robotid = execution_plan.action(i).robotid();
    for (int parentID: execution_plan.action(i).parents()) {
      act.parents.push_back(parentID);
    }
    for (std::string waypoint : execution_plan.action(i).waypoints()) {
      act.waypoints.push_back(waypoint);
    }
    actions.push_back(act);
  }
  return actions;
}

std::vector<plan2_msgs::msg::Method> Plan::getMethods(const planning2::ExecutionPlan& execution_plan){
  std::vector<plan2_msgs::msg::Method> methods;
  for (int i = 0; i < execution_plan.method_size(); i++) {
    plan2_msgs::msg::Method method;
    method.id = execution_plan.method(i).id();
    method.name = execution_plan.method(i).name();
    for (int subtask : execution_plan.method(i).actions_ids()) {
      method.substasks.push_back(subtask);
    }
    methods.push_back(method) ;
  }
  return methods;

}


ExecutionPlan Plan::ParseFile(std::string file){
ExecutionPlan execution_plan;
  { 
    // Read the existing Execution Plan.
    fstream input(file, ios::in | ios::binary);
    if (!execution_plan.ParseFromIstream(&input)) {
      cerr << "Failed to parse Execution Plan." << endl;
      return execution_plan;
    }
  }
  google::protobuf::ShutdownProtobufLibrary();
  return execution_plan;
}



} //end namespace planning2





