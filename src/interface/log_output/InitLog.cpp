/**
 * @file InitLog.cpp
 * @brief Initialize function for Log output
 */

#include "InitLog.hpp"

#include <interface/initialize/initialize_file_access.hpp>

Logger* InitLog(std::string file_name) {
  IniAccess ini_file(file_name);

  std::string log_file_path = ini_file.ReadString("SIMULATION_SETTINGS", "log_file_save_directory");
  bool log_ini = ini_file.ReadEnable("SIMULATION_SETTINGS", "save_initialize_files");

  Logger* log = new Logger("default.csv", log_file_path, file_name, log_ini, true);

  return log;
}

Logger* InitLogMC(std::string file_name, bool enable) {
  IniAccess ini_file(file_name);

  std::string log_file_path = ini_file.ReadString("SIMULATION_SETTINGS", "log_file_save_directory");
  bool log_ini = ini_file.ReadEnable("SIMULATION_SETTINGS", "save_initialize_files");

  Logger* log = new Logger("mont.csv", log_file_path, file_name, log_ini, enable);

  return log;
}
