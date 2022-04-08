#pragma once
#include <string>
#include <vector>

#include "../Interface/LogOutput/Logger.h"

struct SimulationConfig {
  std::string ini_base_fname_;
  Logger* main_logger_;
  int num_of_simulated_spacecraft_;
  std::vector<std::string> sat_file_;
  std::string gs_file_;
  std::string inter_sat_comm_file_;
  std::string gnss_file_;

  ~SimulationConfig() { delete main_logger_; }
};
