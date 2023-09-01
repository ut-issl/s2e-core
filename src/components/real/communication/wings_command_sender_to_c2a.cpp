/*
 * @file wings_command_sender_to_c2a.cpp
 * @brief A component to send command using WINGS operation file
 */

#include "wings_command_sender_to_c2a.hpp"

#include <library/initialize/initialize_file_access.hpp>

void WingsCommandSenderToC2a::MainRoutine(const int time_count) {
  UNUSED(time_count);
  if (wait_s_ <= 0.0)
  {
    wait_s_ = (double)wings_operation_file_.ExecuteNextLine();
  }
  else{
    wait_s_ -= step_width_s_;
  }
 }

WingsCommandSenderToC2a InitWingsCommandSenderToC2a(ClockGenerator* clock_generator, const double compo_update_step, const std::string initialize_file){
  IniAccess ini_access(initialize_file);
  std::string section = "WINGS_COMMAND_SENDER_TO_C2A_";

  int prescaler = ini_access.ReadInt(section.c_str(), "prescaler");
  if (prescaler <= 1) prescaler = 1;

  double step_width_s = (double)prescaler * compo_update_step;

  std::string c2a_command_data_base_file = ini_access.ReadString(section.c_str(), "c2a_command_database_file");
  std::string wings_operation_file = ini_access.ReadString(section.c_str(), "wings_operation_file");

  return WingsCommandSenderToC2a(prescaler, clock_generator, step_width_s, c2a_command_data_base_file, wings_operation_file);
}
