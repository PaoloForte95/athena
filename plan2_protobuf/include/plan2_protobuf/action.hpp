#ifndef PLAN2_PROTOBUF__ACTION_HPP
#define PLAN2_PROTOBUF__ACTION_HPP

#include <vector>
#include <memory>
#include <string>



namespace plan2_protobuf
{

class Action
{
public:

   Action();

   Action(const int &ID, const std::string &name);

  ~Action();

    
    /**
     * @brief Set the action ID
     * 
     * @param ID The ID to associate to the action.
     */
    void setID(int ID);

    /**
     * @brief Set the Action Name object
     * 
     * @param name The name to associate with the action.
     */
    void setName(std::string name);

     /**
      * @brief Add a parent to the action 
      * 
      * @param parentID The ID of the parent to add.
      */
    void addParent(int parentID);

    /**
     * @brief Add a list of parents to the action
     * 
     * @param parents_ids The list of IDs of the parents to add.
     */
    void addParents(std::vector<int> parents_ids);

    /**
     * @brief Get the Parent at the specific index
     * 
     * @param index the index of parent to get
     * @return The ID of the parent at the given index or -1 if out of index
     */
    int GetParent(int index);

    /**
     * @brief Get the Next Parent in the list
     * 
     * @return The ID of the next parent in the list
     */
    int GetNextParent();

     /**
      * @brief Get all the parents of the action
      * 
      * @return The list of IDs of the parents associated with the action.
      */

    std::vector<int> GetParents();

    /**
     * @brief Check if the action has been already executed. 
     * 
     * @return true if the action was executed correctly, @return false otherwise
     */
    bool isExecuted();

    /**
     * @brief Change the status of the action to executed.
     * 
     */
    void isFinished();

    /**
     * @brief Print the action information.
     * 
     */
    std::string toString();


protected:

  int ID_;
  std::string name_;
  std::vector<int> parents_;
  int curret_parent_ID_;
  bool executed_;
};

}

#endif // PLAN2_PROTOBUF__ACTION_HPP
