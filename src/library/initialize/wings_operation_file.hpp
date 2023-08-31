/**
 * @file wings_operation_file.hpp
 * @brief Classes and functions to read and manage WINGS's operation file (.ops)
 */

#ifndef S2E_LIBRARY_INITIALIZE_WINGS_OPERATION_FILE_HPP_
#define S2E_LIBRARY_INITIALIZE_WINGS_OPERATION_FILE_HPP_

#include <string>
#include <fstream>

#include "c2a_command_database.hpp"

/**
 * @class WingsOperationFile
 * @brief A class to handle WINGS operation file
 */
class WingsOperationFile {
 public:
  /**
   * @fn WingsOperationFile
   * @brief Constructor
   * @param[in] file_path: File path of the operation file
   */
  WingsOperationFile(const std::string file_path, const C2aCommandDatabase& command_database);

  /**
   * @fn ExecuteNextLine
   * @brief Execute next line command
   * @return Required wait time for next execution [s]
   */
  size_t ExecuteNextLine();

 private:
   std::vector<std::string> lines_;  //!!< List of read operation command line
   const C2aCommandDatabase& command_database_; //!< C2A command database

  /**
   * @fn AnalyzeC2aCommand
   * @brief Analyze C2A command Line
   * @param[in] tokens: command line after space separation
   */
  void AnalyzeC2aCommand(const std::vector<std::string> tokens);
};

#endif  // S2E_LIBRARY_INITIALIZE_WINGS_OPERATION_FILE_HPP_
