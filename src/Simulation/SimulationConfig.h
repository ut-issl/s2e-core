#pragma once
#include "../Interface/LogOutput/Logger.h"

struct SimulationConfig
{
  string ini_base_fname_;
  Logger* main_logger_;
  vector<string> sat_file_;
  string gs_file_;
  string inter_sat_comm_file_;
};
