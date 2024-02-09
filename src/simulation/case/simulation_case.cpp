/**
 * @file simulation_case.cpp
 * @brief Base class to define simulation scenario
 */

#include "simulation_case.hpp"

#include <setting_file_reader/initialize_file_access.hpp>
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
  if (monte_carlo_simulator.IsEnabled() == false) {
    // Monte Carlo simulation is disabled
    simulation_configuration_.main_logger_ = InitLog(initialize_base_file);
  } else {
    // Monte Carlo Simulation is enabled
    std::string log_file_name = "default" + std::to_string(monte_carlo_simulator.GetNumberOfExecutionsDone()) + ".csv";

    IniAccess ini_file(initialize_base_file);
    bool save_ini_files = ini_file.ReadEnable("SIMULATION_SETTINGS", "save_initialize_files");

    simulation_configuration_.main_logger_ =
        new Logger(log_file_name, log_path, initialize_base_file, save_ini_files, monte_carlo_simulator.GetSaveLogHistoryFlag());
  }
  // Initialize Simulation Configuration
  InitializeSimulationConfiguration(initialize_base_file);
}

SimulationCase::~SimulationCase() { delete global_environment_; }

void SimulationCase::Initialize() {
  // Target Objects Initialize
  InitializeTargetObjects();

  // Write headers to the log
  simulation_configuration_.main_logger_->WriteHeaders();

  // Start the simulation
  std::cout << "\nSimulationDateTime \n";
  global_environment_->GetSimulationTime().PrintStartDateTime();
}

void SimulationCase::Main() {
  global_environment_->Reset();  // for MonteCarlo Simulation
  while (!global_environment_->GetSimulationTime().GetState().finish) {
    // Logging
    if (global_environment_->GetSimulationTime().GetState().log_output) {
      simulation_configuration_.main_logger_->WriteValues();
    }

    // Global Environment Update
    global_environment_->Update();

    // Target Objects Update
    UpdateTargetObjects();

    // Debug output
    if (global_environment_->GetSimulationTime().GetState().disp_output) {
      std::cout << "Progress: " << global_environment_->GetSimulationTime().GetProgressionRate() << "%\r";
    }
  }
}

std::string SimulationCase::GetLogHeader() const {
  std::string str_tmp = "";

  return str_tmp;
}

std::string SimulationCase::GetLogValue() const {
  std::string str_tmp = "";

  return str_tmp;
}

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
  global_environment_->LogSetup(*(simulation_configuration_.main_logger_));
}