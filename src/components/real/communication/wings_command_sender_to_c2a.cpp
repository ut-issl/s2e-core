/*
 * @file wings_command_sender_to_c2a.cpp
 * @brief A component to send C2A command using WINGS operation file
 */

#include "wings_command_sender_to_c2a.hpp"

#include <library/initialize/initialize_file_access.hpp>
#include <library/utilities/macros.hpp>
#include <regex>

#ifdef USE_C2A
#if C2A_CORE_VER_MAJOR == 4
// c2a-core v4
#include "src_core/TlmCmd/common_cmd_packet_util.h"
#elif C2A_CORE_VER_MAJOR <= 3
// c2a-core <= v3
#include "src_core/TlmCmd/common_cmd_packet_util.h"
#else
#error "c2a-core version is not supported"
#endif  // c2a-core version header
#endif  // USE_C2A

void WingsCommandSenderToC2a::MainRoutine(const int time_count) {
  UNUSED(time_count);
  if (is_end_of_line_ == true) return;
  if (wait_s_ <= 0.0) {
    std::string line = wings_operation_file_.GetLatestLine();
    if (line == "EOL") {
      is_end_of_line_ = true;
    } else {
      ExecuteCommandLine(line);
    }
  } else {
    wait_s_ -= step_width_s_;
  }
}

void WingsCommandSenderToC2a::ExecuteCommandLine(const std::string line) {
  // Separate with space
  std::istringstream token_stream(line);
  std::string token;
  std::vector<std::string> tokens;
  while (token_stream >> token) {
    tokens.push_back(token);
  }

  // Handle WINGS commands
  if (tokens[0].find("wait_sec") == 0) {
    // wait process
    wait_s_ = std::stod(tokens[1]);
  } else if (tokens[0].find("check_value") == 0) {
    // TODO: Support check_value
    wait_s_ = 0;
  } else if (tokens[0].find("let") == 0) {
    // TODO: Support let command
    wait_s_ = 0;
  } else {
    wait_s_ = 0;
    AnalyzeC2aCommand(tokens);
  }

  return;
}

void WingsCommandSenderToC2a::AnalyzeC2aCommand(const std::vector<std::string> tokens) {
#ifdef USE_C2A
  // Recognize command
  size_t first_underscore_position = tokens[0].find('_');
  if (first_underscore_position == std::string::npos) return;
  size_t first_dot_position = tokens[0].find('.');
  if (first_underscore_position == std::string::npos) return;

  std::string target_name = tokens[0].substr(0, first_underscore_position);
  // TODO check target?
  std::string command_type = tokens[0].substr(first_underscore_position + 1, first_dot_position - (first_underscore_position + 1));
  std::string command_name = tokens[0].substr(first_dot_position + 1);

  // Get command code
  C2aCommandInformation cmd_info = c2a_command_database_.GetCommandInformation(command_name);
  if (cmd_info.GetCommandName() == "Error") {
    // TODO: error handling
  }
  CMD_CODE cmd_id = (CMD_CODE)cmd_info.GetCommandId();

  // Get arguments
  std::vector<std::string> arguments;
  for (size_t i = 1; i < tokens.size(); i++) {
    if (tokens[i].find("{") == 0) return; // let command is not supported now TODO: support let command
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
#else
  UNUSED(tokens);
#endif
}

WingsCommandSenderToC2a InitWingsCommandSenderToC2a(ClockGenerator* clock_generator, const double compo_update_step_s,
                                                    const std::string initialize_file) {
  IniAccess ini_access(initialize_file);
  std::string section = "WINGS_COMMAND_SENDER_TO_C2A";

  int prescaler = ini_access.ReadInt(section.c_str(), "prescaler");
  if (prescaler <= 1) prescaler = 1;

  double step_width_s = (double)prescaler * compo_update_step_s;

  std::string c2a_command_data_base_file = ini_access.ReadString(section.c_str(), "c2a_command_database_file");
  std::string wings_operation_file = ini_access.ReadString(section.c_str(), "wings_operation_file");

  return WingsCommandSenderToC2a(prescaler, clock_generator, step_width_s, c2a_command_data_base_file, wings_operation_file);
}
