/**
 * @file c2a_command_database.hpp
 * @brief
 */

#ifndef S2E_LIBRARY_INITIALIZE_C2_COMMAND_DATABASE_HPP_
#define S2E_LIBRARY_INITIALIZE_C2_COMMAND_DATABASE_HPP_

#include <string>
#include <vector>
#include <map>

enum class C2aArgumentType{
  kUint8t,
  kUint16t,
  kUint32t,
  kUint64t,
  kInt8t,
  kInt16t,
  kInt32t,
  kInt64t,
  kFloat,
  kDouble,
  kRaw,
  kError
};

/**
 * @class C2aCommandDatabase
 * @brief
 */
class C2aCommandInformation {
 public:
  C2aCommandInformation(){}
  C2aCommandInformation(const std::string cmd_db_line);

  // Getter
  inline std::string GetCommandName() const {return command_name_;}
  inline size_t GetCommandId() const {return command_id_;}
  inline size_t GetNumberOfArgument() const { return argument_type_info_.size(); }
  C2aArgumentType GetArgumentType(const size_t argument_id);

 private:
  std::string command_name_ = "Error";
  size_t command_id_ = 0;
  size_t number_of_arguments_ = 0;
  std::vector<C2aArgumentType> argument_type_info_;

  C2aArgumentType ConvertArgumentType(const std::string type);
};

/**
 * @class C2aCommandDatabase
 * @brief
 */
class C2aCommandDatabase {
 public:
  /**
   * @fn C2aCommandDatabase
   * @brief Constructor
   * @param[in] file_path: File path of the database file
   */
  C2aCommandDatabase(const std::string file_path);

  // Getter
  inline C2aCommandInformation GetCommandInformation(const std::string command_name) const {
    auto itr = command_map_.find(command_name);
    if (itr != command_map_.end())
    {
      return itr->second;
    }
    else
    {
      return C2aCommandInformation();
    }
  }

 private:
  std::map<std::string, C2aCommandInformation> command_map_;
};

#endif  // S2E_LIBRARY_INITIALIZE_C2_COMMAND_DATABASE_HPP_
