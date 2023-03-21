/**
 * @file simulation_case.cpp
 * @brief Base class to define simulation scenario
 */

#include "simulation_case.hpp"

#include <library/initialize/initialize_file_access.hpp>
#include <library/logger/initialize_log.hpp>
#include <string>

SimulationCase::SimulationCase(const std::string initialize_base_file) {
  // Initialize Log
  simulation_configuration_.main_logger_ = InitLog(initialize_base_file);

  // Initialize Simulation Configuration
  InitializeSimulationConfiguration(initialize_base_file);
}

SimulationCase::SimulationCase(const std::string initialize_base_file, const MonteCarloSimulationExecutor& monte_carlo_simulator,
                               const std::string log_path) {
  // Initialize Log
  // Log for Monte Carlo Simulation
  std::string log_file_name = "default" + std::to_string(monte_carlo_simulator.GetNumberOfExecutionsDone()) + ".csv";
  // TODO: Consider that `enable_inilog = false` is fine or not?
  simulation_configuration_.main_logger_ =
      new Logger(log_file_name, log_path, initialize_base_file, false, monte_carlo_simulator.GetSaveLogHistoryFlag());

  // Initialize Simulation Configuration
  InitializeSimulationConfiguration(initialize_base_file);
}

SimulationCase::~SimulationCase() { delete global_environment_; }

void SimulationCase::InitializeSimulationConfiguration(const std::string initialize_base_file) {
  // Initialize
  IniAccess simulation_base_ini = IniAccess(initialize_base_file);
  const char* section = "SIMULATION_SETTINGS";
  simulation_configuration_.initialize_base_file_name_ = initialize_base_file;

  // Spacecraft
  simulation_configuration_.number_of_simulated_spacecraft_ = simulation_base_ini.ReadInt(section, "number_of_simulated_spacecraft");
  simulation_configuration_.spacecraft_file_list_ = simulation_base_ini.ReadStrVector(section, "spacecraft_file");

  // Ground Station
  simulation_configuration_.number_of_simulated_ground_station_ = simulation_base_ini.ReadInt(section, "number_of_simulated_ground_station");
  simulation_configuration_.ground_station_file_list_ = simulation_base_ini.ReadStrVector(section, "ground_station_file");

  // Others
  simulation_configuration_.inter_sc_communication_file_ = simulation_base_ini.ReadString(section, "inter_sat_comm_file");
  simulation_configuration_.gnss_file_ = simulation_base_ini.ReadString(section, "gnss_file");

  // Global Environment
  global_environment_ = new GlobalEnvironment(&simulation_configuration_);
}