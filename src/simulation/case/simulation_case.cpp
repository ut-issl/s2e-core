/**
 * @file simulation_case.cpp
 * @brief Base class to define simulation scenario
 */

#include "simulation_case.hpp"

#include <library/initialize/initialize_file_access.hpp>
#include <library/logger/initialize_log.hpp>
#include <string>

SimulationCase::SimulationCase(std::string ini_base) {
  IniAccess simbase_ini = IniAccess(ini_base);
  const char* section = "SIMULATION_SETTINGS";
  sim_config_.initialize_base_file_name_ = ini_base;
  sim_config_.main_logger_ = InitLog(sim_config_.initialize_base_file_name_);
  sim_config_.number_of_simulated_spacecraft_ = simbase_ini.ReadInt(section, "number_of_simulated_spacecraft");
  sim_config_.spacecraft_file_list_ = simbase_ini.ReadStrVector(section, "spacecraft_file");

  sim_config_.number_of_simulated_ground_station_ = simbase_ini.ReadInt(section, "number_of_simulated_ground_station");
  sim_config_.ground_station_file_list_ = simbase_ini.ReadStrVector(section, "ground_station_file");

  sim_config_.inter_sc_communication_file_ = simbase_ini.ReadString(section, "inter_sat_comm_file");
  sim_config_.gnss_file_ = simbase_ini.ReadString(section, "gnss_file");
  global_environment_ = new GlobalEnvironment(&sim_config_);
}
SimulationCase::SimulationCase(std::string ini_base, const MonteCarloSimulationExecutor& monte_carlo_simulator, const std::string log_path) {
  IniAccess simbase_ini = IniAccess(ini_base);
  const char* section = "SIMULATION_SETTINGS";
  sim_config_.initialize_base_file_name_ = ini_base;
  // Log for Monte Carlo Simulation
  std::string log_file_name = "default" + std::to_string(monte_carlo_simulator.GetNumberOfExecutionsDone()) + ".csv";
  // ToDo: Consider that `enable_inilog = false` is fine or not?
  sim_config_.main_logger_ = new Logger(log_file_name, log_path, ini_base, false, monte_carlo_simulator.GetSaveLogHistoryFlag());
  sim_config_.number_of_simulated_spacecraft_ = simbase_ini.ReadInt(section, "number_of_simulated_spacecraft");
  sim_config_.spacecraft_file_list_ = simbase_ini.ReadStrVector(section, "spacecraft_file");

  sim_config_.number_of_simulated_ground_station_ = simbase_ini.ReadInt(section, "number_of_simulated_ground_station");
  sim_config_.ground_station_file_list_ = simbase_ini.ReadStrVector(section, "ground_station_file");

  sim_config_.inter_sc_communication_file_ = simbase_ini.ReadString(section, "inter_sat_comm_file");
  sim_config_.gnss_file_ = simbase_ini.ReadString(section, "gnss_file");
  // Global Environment
  global_environment_ = new GlobalEnvironment(&sim_config_);
}
SimulationCase::~SimulationCase() { delete global_environment_; }
