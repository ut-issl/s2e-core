/**
 * @file c2a_command_database.hpp
 * @brief Classes and functions to read and manage C2A command database
 */

#ifndef S2E_LIBRARY_INITIALIZE_C2A_COMMAND_DATABASE_HPP_
#define S2E_LIBRARY_INITIALIZE_C2A_COMMAND_DATABASE_HPP_

#include <stdint.h>

#include <map>
#include <string>
#include <vector>

namespace s2e::setting_file_reader {

/**
 * @enum C2aArgumentType
 * @brief Argument type used in C2A command
 */
enum class C2aArgumentType { kUint8t, kUint16t, kUint32t, kUint64t, kInt8t, kInt16t, kInt32t, kInt64t, kFloat, kDouble, kRaw, kError };

/**
 * @class C2aCommandInformation
 * @brief Information of C2A command
 */
class C2aCommandInformation {
 public:
  /**
   * @fn C2aCommandInformation
   * @brief Constructor
   */
  C2aCommandInformation() {}

  /**
   * @fn C2aCommandInformation
   * @brief Constructor
   * @param[in] cmd_db_line: A line in command database
   */
  C2aCommandInformation(const std::string cmd_db_line);

  // Getter
  /**
   * @fn GetCommandName
   * @brief Return command name
   */
  inline std::string GetCommandName() const { return command_name_; }
  /**
   * @fn GetCommandId
   * @brief Return command ID
   */
  inline size_t GetCommandId() const { return command_id_; }
  /**
   * @fn GetNumberOfArgument
   * @brief Return number of argument
   */
  inline size_t GetNumberOfArgument() const { return argument_type_info_.size(); }
  /**
   * @fn GetArgumentType
   * @brief Return argument type
   */
  C2aArgumentType GetArgumentType(const size_t argument_id);

 private:
  std::string command_name_ = "Error";               //!< Command name
  size_t command_id_ = 0;                            //!< Command ID
  size_t number_of_arguments_ = 0;                   //!< Number of arguments
  std::vector<C2aArgumentType> argument_type_info_;  //!< List of argument type

  /**
   * @fn ConvertArgumentType
   * @brief Convert type expressed by string to enum
   * @param[in] type: type information
   */
  C2aArgumentType ConvertArgumentType(const std::string type);
};

/**
 * @class C2aCommandDatabase
 * @brief A class to handle C2A command database
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
  /**
   * @fn GetCommandInformation
   * @brief Return C2A command information
   * @param[in] command_name: Command Name
   */
  inline C2aCommandInformation GetCommandInformation(const std::string command_name) const {
    auto itr = command_map_.find(command_name);
    if (itr != command_map_.end()) {
      return itr->second;
    } else {
      return C2aCommandInformation();
    }
  }

 private:
  std::map<std::string, C2aCommandInformation> command_map_;  //!< Command database
};

/**
 * @fn DecodeC2aCommandArgument
 * @brief Decode argument value expressed by string as C2A command argument
 * @param[in] type: Argument type
 * @param[in] argument_string: Argument value expressed as string
 * @param[out] param: Decoded value
 * @param[out] size_param: Size of decoded value [byte]
 */
void DecodeC2aCommandArgument(const C2aArgumentType type, const std::string argument_string, uint8_t* param, size_t& size_param);

}  // namespace s2e::setting_file_reader

#endif  // S2E_LIBRARY_INITIALIZE_C2A_COMMAND_DATABASE_HPP_
