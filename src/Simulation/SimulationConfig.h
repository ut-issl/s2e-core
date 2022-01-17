#pragma once
#include "../Interface/LogOutput/Logger.h"
#include <string>
#include <vector>

struct SimulationConfig {
  std::string ini_base_fname_;
  Logger *main_logger_;
  int num_of_simulated_spacecraft_;
  std::vector<std::string> sat_file_;
  std::string gs_file_;
  std::string inter_sat_comm_file_;
  std::string gnss_file_;
};
