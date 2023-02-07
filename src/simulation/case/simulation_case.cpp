/**
 * @file simulation_case.cpp
 * @brief Base class to define simulation scenario
 */

#include "simulation_case.hpp"

#include <interface/initialize/initialize_file_access.hpp>

#include <interface/log_output/InitLog.hpp>
#include <string>

SimulationCase::SimulationCase(std::string ini_base) {
  IniAccess simbase_ini = IniAccess(ini_base);
  const char* section = "SIMULATION_SETTINGS";
  sim_config_.ini_base_fname_ = ini_base;
  sim_config_.main_logger_ = InitLog(sim_config_.ini_base_fname_);
  sim_config_.num_of_simulated_spacecraft_ = simbase_ini.ReadInt(section, "number_of_simulated_spacecraft");
  sim_config_.sat_file_ = simbase_ini.ReadStrVector(section, "spacecraft_file");
  sim_config_.gs_file_ = simbase_ini.ReadString(section, "ground_station_file");
  sim_config_.inter_sat_comm_file_ = simbase_ini.ReadString(section, "inter_sat_comm_file");
  sim_config_.gnss_file_ = simbase_ini.ReadString(section, "gnss_file");
  glo_env_ = new GlobalEnvironment(&sim_config_);
}
SimulationCase::SimulationCase(std::string ini_base, const MCSimExecutor& mc_sim, const std::string log_path) {
  IniAccess simbase_ini = IniAccess(ini_base);
  const char* section = "SIMULATION_SETTINGS";
  sim_config_.ini_base_fname_ = ini_base;
  // Log for Monte Carlo Simulation
  std::string log_file_name = "default" + std::to_string(mc_sim.GetNumOfExecutionsDone()) + ".csv";
  // ToDo: Consider that `enable_inilog = false` is fine or not?
  sim_config_.main_logger_ = new Logger(log_file_name, log_path, ini_base, false, mc_sim.LogHistory());
  sim_config_.num_of_simulated_spacecraft_ = simbase_ini.ReadInt(section, "number_of_simulated_spacecraft");
  sim_config_.sat_file_ = simbase_ini.ReadStrVector(section, "spacecraft_file");
  sim_config_.gs_file_ = simbase_ini.ReadString(section, "ground_station_file");
  sim_config_.inter_sat_comm_file_ = simbase_ini.ReadString(section, "inter_sat_comm_file");
  sim_config_.gnss_file_ = simbase_ini.ReadString(section, "gnss_file");
  // Global Environment
  glo_env_ = new GlobalEnvironment(&sim_config_);
}
SimulationCase::~SimulationCase() { delete glo_env_; }
