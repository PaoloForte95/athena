
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>


#include "plan2_protobuf/execution_plan.hpp"

using namespace std;

namespace planning2{



std::vector<planning2::Action> Plan::GetActions(const planning2::ExecutionPlan& execution_plan) {
  std::vector<planning2::Action> actions;
  for (int i = 0; i < execution_plan.action_size(); i++) {
    
    const Action& protoAction = execution_plan.action(i);
  
    for (int j = 0; j < protoAction.waypoints().size(); j++) {
      auto waypoint = protoAction.waypoints(j);
    }
    for (int j = 0; j < protoAction.parents_size(); j++) {
      auto parentID = protoAction.parents(j);
    }
    actions.push_back(protoAction);
  }
return actions;
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





