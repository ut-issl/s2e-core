/**
 * @file wings_operation_file.hpp
 * @brief Classes and functions to read and manage WINGS's operation file (.ops)
 */

#ifndef S2E_LIBRARY_INITIALIZE_WINGS_OPERATION_FILE_HPP_
#define S2E_LIBRARY_INITIALIZE_WINGS_OPERATION_FILE_HPP_

#include <fstream>
#include <string>

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
  WingsOperationFile(const std::string file_path);

  /**
   * @fn GetLatestLine
   * @brief Return latest line command
   */
  std::string GetLatestLine();

 private:
  std::vector<std::string> lines_;  //!!< List of read operation command line
  size_t line_pointer_ = 0;         //!< Line pointer
};

#endif  // S2E_LIBRARY_INITIALIZE_WINGS_OPERATION_FILE_HPP_
