#pragma once

#include <Interface/LogOutput/Logger.h>

Logger* InitLog(std::string file_name);
Logger* InitLogMC(std::string file_name, bool enable);
