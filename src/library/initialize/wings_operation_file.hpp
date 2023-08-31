/**
 * @file wings_operation_file.hpp
 * @brief Classes and functions to read and manage WINGS's operation file (.ops)
 */

#ifndef S2E_LIBRARY_INITIALIZE_WINGS_OPERATION_FILE_HPP_
#define S2E_LIBRARY_INITIALIZE_WINGS_OPERATION_FILE_HPP_

#include <string>

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

 private:

};

#endif  // S2E_LIBRARY_INITIALIZE_WINGS_OPERATION_FILE_HPP_
