#include <plan2_protobuf/method.hpp>

#include <limits>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <iterator>

namespace plan2_protobuf
{
Method::Method(const int & ID)
{ID_ = ID;}

Method::Method(
  const int & ID,
  const std::string & name)
{
 ID_ = ID;
 name_ = name;
 executed_ = false;
}

Method::~Method()
{

}

void Method::addSubtask(int subtask_id){
     auto itr = std::find(subtasks_.begin(), subtasks_.end(), subtask_id);
    //Add only if absent
    if (itr == subtasks_.end()){
        std::cout << "Adding Subtask ID " << subtask_id  << " ";
        subtasks_.push_back(subtask_id);
    }
    subtasks_.push_back(subtask_id);
}

void Method::setName(std::string name){
    name_ = name;
}

void Method::addSubtasks(std::vector<int> subtasks_ids){

    for(auto subtask : subtasks_ids){
        addSubtask(subtask);
    }
    
}
std::vector<int> Method::GetSubtasks(){
    return subtasks_;
}


bool Method::isExecuted(){
    return executed_;
}

void Method::isFinished(){
     executed_ = true;
}

inline std::string BoolToString(bool b)
{
  return b ? "true" : "false";
}

std::string Method::toString(){
    std::string info;
    std::stringstream subtasks_ids;
    std::copy(subtasks_.begin(), subtasks_.end(), std::ostream_iterator<int>(subtasks_ids, " "));
    info += "Action: name" + name_ + " id:" + std::to_string(ID_) + "is executed? " + BoolToString(executed_) + " subtasks IDs list: " + subtasks_ids.str().c_str();
    return info;
}
} //end namespace plan2_protobuf