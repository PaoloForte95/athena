#ifndef ATHENA_PROTOBUF__EXECUTION_PLAN_HPP
#define ATHENA_PROTOBUF__EXECUTION_PLAN_HPP

#include <iostream>
#include <string>
#include <fstream>

#include "athena_protobuf/executionplan.pb.h"
#include "athena_protobuf/action.hpp"
#include "athena_protobuf/method.hpp"

#include "athena_msgs/msg/action.hpp"
#include "athena_msgs/msg/method.hpp"


namespace athena_protobuf{

class Plan 
{
public:   

    /**
     * @brief Iterates though all the actions in the ExecutionPlan
     * 
     * @param execution_plan the data file that contains the execution plan.
     */
    std::vector<athena_msgs::msg::Action> GetActions(const athena::ProtoExecutionPlan& execution_plan);


    /**
     * @brief Iterates though all the methods in the ExecutionPlan
     * 
     * @param execution_plan the data file that contains the execution plan.
     */
    std::vector<athena_msgs::msg::Method> getMethods(const athena::ProtoExecutionPlan& execution_plan);

    /**
     * @brief Parse  the dataFile computed by java
     * 
     * @param file the name of the data file generated by java
     * @return The parsed execution plan  
     */
    athena::ProtoExecutionPlan ParseFile(std::string file);

}; //End Class
} 


#endif
