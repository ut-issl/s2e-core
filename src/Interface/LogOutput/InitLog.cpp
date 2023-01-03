#include "InitLog.hpp"

#include <Interface/InitInput/IniAccess.h>

Logger* InitLog(std::string file_name, const std::string& log_dir_name) {
  IniAccess ini_file(file_name);

  std::string log_root_dir = ini_file.ReadString("SIM_SETTING", "log_file_path");
  bool log_ini = ini_file.ReadBoolean("SIM_SETTING", "log_inifile");

  Logger* log = new Logger("default.csv", log_root_dir, file_name, log_ini, true, log_dir_name);

  return log;
}

Logger* InitLogMC(std::string file_name, bool enable, const std::string& log_dir_name) {
  IniAccess ini_file(file_name);

  std::string log_root_dir = ini_file.ReadString("SIM_SETTING", "log_file_path");
  bool log_ini = ini_file.ReadBoolean("SIM_SETTING", "log_inifile");

  Logger* log = new Logger("mont.csv", log_root_dir, file_name, log_ini, enable, log_dir_name);

  return log;
}
