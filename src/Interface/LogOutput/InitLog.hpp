#pragma once

#include <Interface/LogOutput/Logger.h>

Logger* InitLog(std::string file_name, const std::string& sim_name = "");
Logger* InitLogMC(std::string file_name, bool enable, const std::string& sim_name = "");
