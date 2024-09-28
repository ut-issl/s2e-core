/**
 * @file wings_operation_file.cpp
 * @brief Classes and functions to read and manage WINGS's operation file (.ops)
 */

#include "wings_operation_file.hpp"

#include <iostream>

namespace s2e::setting_file_reader {

WingsOperationFile::WingsOperationFile(const std::string file_path) {
  // File open
  std::ifstream file(file_path);
  if (!file.is_open()) {
    std::cerr << "WINGS Operation file open error." << std::endl;
  }

  // Read all lines
  std::string line;
  while (std::getline(file, line)) {
    // Remove pose point
    if (!line.empty() && line[0] == '.') {
      line = line.substr(1);
    }
    // Remove space
    if (!line.empty() && line[0] == ' ') {
      line = line.substr(1);
    }
    // Remove comment
    size_t comment_position = line.find('#');
    if (comment_position != std::string::npos) {
      line = line.substr(0, comment_position);
    }
    // add line
    if (line.length() > 2) {
      lines_.push_back(line);
    }
  }
}

std::string WingsOperationFile::GetLatestLine() {
  if (line_pointer_ >= lines_.size()) return "EOL";

  std::string line = lines_[line_pointer_];
  line_pointer_++;

  return line;
}

}  // namespace s2e::setting_file_reader
