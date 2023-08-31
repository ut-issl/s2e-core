/**
 * @file wings_operation_file.cpp
 * @brief Classes and functions to read and manage WINGS's operation file (.ops)
 */

#include "wings_operation_file.hpp"

#include <iostream>
#include <regex>

#if C2A_CORE_VER_MAJOR == 4
// c2a-core v4
#elif C2A_CORE_VER_MAJOR <= 3
// c2a-core <= v3
#include "src_core/TlmCmd/common_cmd_packet_util.h"
#else
#error "c2a-core version is not supported"
#endif  // c2a-core version header

WingsOperationFile::WingsOperationFile(const std::string file_path, const C2aCommandDatabase& command_database)
    : command_database_(command_database) {
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

size_t WingsOperationFile::ExecuteNextLine() {
  std::string input_line = lines_[0];

  // Separate with space
  std::istringstream token_stream(input_line);
  std::string token;
  std::vector<std::string> tokens;
  while (token_stream >> token) {
    tokens.push_back(token);
  }

  if (tokens[0].find("wait_sec") == 0) {
    // wait process
    size_t wait_time_sec = (uint32_t)std::stoi(tokens[0]);
    return wait_time_sec;
  }

  if (tokens[0].find("check_value") == 0) {
    // TODO
    return 0;
  }

  AnalyzeC2aCommand(tokens);
  return 0;
}

void WingsOperationFile::AnalyzeC2aCommand(const std::vector<std::string> tokens) {
  // Recognize command
  size_t first_underscore_position = tokens[0].find('_');
  if (first_underscore_position == std::string::npos) return;
  size_t first_dot_position = tokens[0].find('.');
  if (first_underscore_position == std::string::npos) return;

  std::string target_name = tokens[0].substr(0, first_underscore_position);
  // TODO check target?
  std::string command_type = tokens[0].substr(first_underscore_position + 1, first_dot_position);
  std::string command_name = tokens[0].substr(first_dot_position + 1);

  // Get command code
  C2aCommandInformation cmd_info = command_database_.GetCommandInformation(command_name);
  if (cmd_info.GetCommandName() == "Error") {
    // TODO: error handling
  }
  CMD_CODE cmd_id = (CMD_CODE)cmd_info.GetCommandId();

  // Get arguments
  std::vector<std::string> arguments;
  for (size_t i = 1; i < tokens.size(); i++) {
    arguments.push_back(tokens[i]);
  }
  // Check number of arguments
  if (arguments.size() != cmd_info.GetNumberOfArgument()) {
    // TODO: error handling
  }

  // Decode arguments
  // Read TI
  uint32_t ti = 0;
  if (command_type == "TL" || command_type == "BL") {
    ti = (uint32_t)std::stoi(arguments[0]);
    arguments.erase(arguments.begin());
  }
  // Read arguments
  uint8_t param[CSP_MAX_LEN];
  uint16_t param_len = 0;
  for (size_t arg_num = 0; arg_num < arguments.size(); arg_num++) {
    size_t len = 0;
    DecodeC2aCommandArgument(cmd_info.GetArgumentType(arg_num), arguments[arg_num], param + param_len, len);
    param_len += (uint16_t)len;
  }

  // Send command
  if (command_type == "RT") {
    CCP_register_rtc(cmd_id, param, param_len);
  } else if (command_type == "TL") {
    CCP_register_tlc(ti, TLCD_ID_FROM_GS, cmd_id, param, param_len);
  } else if (command_type == "BL") {
    // TODO: BL実装したいが、register_blがない？
  } else {
    // Not reach
  }
}
