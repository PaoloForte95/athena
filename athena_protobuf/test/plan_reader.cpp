
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

#include "gtest/gtest.h"


#include "athena_protobuf/execution_plan.hpp"
#include "athena_protobuf/action.hpp"
#include "athena_protobuf/executionplan.pb.h"


using namespace std;


std::string toString(athena::Action action){
    std::string info;
    std::stringstream parent_ids;
    std::copy(action.parents().begin(), action.parents().end(), std::ostream_iterator<int>(parent_ids, " "));
    std::stringstream waypoints;
    std::copy(action.waypoints().begin(), action.waypoints().end(), std::ostream_iterator<std::string>(waypoints, " "));
    info += "Action: " + action.name() + " id: " + std::to_string(action.id()) +  " parents IDs list: " + parent_ids.str().c_str() + " waypoints: " + waypoints.str().c_str();
    return info;
}



// Main function:  Reads the entire address book from a file and prints all the information inside.
TEST(PlanReaderTest, plan_reader)
{
  // Verify that the version of the library that we linked against is compatible with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  std::string proto_file = "ExePlan.data";
  athena_protobuf::Plan plan;
  athena::ExecutionPlan execution_plan = plan.ParseFile(proto_file);

  auto actions = plan.GetActions(execution_plan);

  for (athena_msgs::msg::Action action: actions){
    cout << action.name << endl;
    cout << action.robotid << endl;
  }


}
