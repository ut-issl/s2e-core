#pragma once
#include "../Interface/LogOutput/Logger.h"

struct SimulationConfig
{
  string ini_base_fname_;
  Logger* main_logger_;
  int num_of_simulated_spacecraft_;
  vector<string> sat_file_;
  string gs_file_;
  string inter_sat_comm_file_;
  string gnss_file_;
};
