#include <athena_protobuf/action.hpp>

#include <limits>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <iterator>

namespace athena_protobuf
{
Action::Action()
{curret_parent_ID_ = 0;}

Action::Action(
  const int & ID,
  const std::string & name)
{
 ID_ = ID;
 name_ = name;
 curret_parent_ID_ = 0;
 executed_ = false;
}

Action::~Action()
{

}

void Action::addParent(int parent_id){
     auto itr = std::find(parents_.begin(), parents_.end(), parent_id);
    //Add only if absent
    if (itr == parents_.end()){
        std::cout << "Adding parent ID " << parent_id  << " ";
        parents_.push_back(parent_id);
    }
    parents_.push_back(parent_id);
}

void Action::setID(int ID){
     ID_ = ID;
}

void Action::setName(std::string name){
    name_ = name;
}

void Action::addParents(std::vector<int> parents_ids){

    for(auto parent : parents_ids){
        addParent(parent);
    }
    
}

int Action::GetParent(int index){
    if(index < parents_.size()){
        return parents_[index];
    }
    return -1;
    
}

int Action::GetNextParent(){
    int parentID = GetParent(curret_parent_ID_);
    if(parentID != -1){
        curret_parent_ID_ +=1;
        return parentID;
    }
    return -1;
}

std::vector<int> Action::GetParents(){
    return parents_;
}


bool Action::isExecuted(){
    return executed_;
}

void Action::isFinished(){
     executed_ = true;
}

inline std::string BoolToString(bool b)
{
  return b ? "true" : "false";
}

std::string Action::toString(){
    std::string info;
    std::stringstream parent_ids;
    std::copy(parents_.begin(), parents_.end(), std::ostream_iterator<int>(parent_ids, " "));
    info += "Action: name" + name_ + " id:" + std::to_string(ID_) + "is executed? " + BoolToString(executed_) + " parents IDs list: " + parent_ids.str().c_str();
    return info;
}
} //end namespace athena_protobuf