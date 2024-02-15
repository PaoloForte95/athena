#ifndef ATHENA_PROTOBUF__METHOD_HPP
#define ATHENA_PROTOBUF__METHOD_HPP

#include <vector>
#include <memory>
#include <string>



namespace athena_protobuf
{

class Method
{
public:

   Method(const int &ID);

   Method(const int &ID, const std::string &name);

  ~Method();

    /**
     * @brief Set the Action Name object
     * 
     * @param name The name to associate with the action.
     */
    void setName(std::string name);

     /**
      * @brief Add a parent to the method, i.e., a method with a precedence constraint on this.
      * 
      * @param ID The ID of the parent method to add.
      */
    void addParent(int ID);


     /**
      * @brief Add a subtask to the method 
      * 
      * @param ID The ID of the subtask to add.
      */
    void addSubtask(int ID);

    /**
     * @brief Add a list of subtasks to the action
     * 
     * @param subtasks_ids The list of IDs of the subtasks to add.
     */
    void addSubtasks(std::vector<int> subtasks_ids);


      /**
      * @brief Get all the subtasks of the method
      * 
      * @return The list of IDs of the subtasks associated with the method.
      */

    std::vector<int> GetSubtasks();


      /**
     * @brief Check if the method has been already executed, namely, all the subtasks have been executed.
     * 
     * @return true if the method was executed correctly, @return false otherwise
     */
    bool isExecuted();

    /**
     * @brief Change the status of the method to executed.
     * 
     */
    void isFinished();


    /**
     * @brief Print the method information.
     * 
     */
    std::string toString();


protected:

  int ID_;
  std::string name_;
  std::vector<int> parents_;
  std::vector<int> subtasks_;
  bool executed_;


};

}

#endif 