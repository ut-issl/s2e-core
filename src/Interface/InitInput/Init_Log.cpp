#include "Initialize.h"

Logger *InitLog(std::string file_name) {
  IniAccess ini_file(file_name);

  std::string log_file_path =
      ini_file.ReadString("SIM_SETTING", "log_file_path");
  bool log_ini = ini_file.ReadBoolean("SIM_SETTING", "log_inifile");

  Logger *log =
      new Logger("default.csv", log_file_path, file_name, log_ini, true);

  return log;
}

Logger *InitLogMC(std::string file_name, bool enable) {
  IniAccess ini_file(file_name);

  std::string log_file_path =
      ini_file.ReadString("SIM_SETTING", "log_file_path");
  bool log_ini = ini_file.ReadBoolean("SIM_SETTING", "log_inifile");

  Logger *log =
      new Logger("mont.csv", log_file_path, file_name, log_ini, enable);

  return log;
}
